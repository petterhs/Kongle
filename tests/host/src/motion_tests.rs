use crate::motion::{Error, Motion, Variant};
use embedded_hal::i2c::{ErrorKind, ErrorType};
use embedded_hal_async::{
    delay::DelayNs,
    i2c::{I2c, Operation},
};
use std::{
    cell::RefCell,
    future::Future,
    rc::Rc,
    sync::Arc,
    task::{Context, Poll, Wake, Waker},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fault;
impl embedded_hal::i2c::Error for Fault {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

struct Sensor {
    id: u8,
    status: u8,
    completion_status: u8,
    registers: [u8; 256],
    features: [u8; 70],
    uploaded: Vec<u8>,
    feature_pointer: usize,
    loaded: bool,
    feature_pointer_published: bool,
    writes: usize,
    fail_upload: bool,
    corrupt_enable: bool,
    corrupt_register: Option<(u8, u8)>,
}
impl Sensor {
    fn new(id: u8) -> Self {
        let mut features = [0; 70];
        features[0x3b] = 0x28; // Detector + activity must be cleared, not inherited.
        features[0x42] = 0x34;
        features[0x43] = 0x12;
        Self {
            id,
            status: 0,
            completion_status: 1,
            registers: [0; 256],
            features,
            uploaded: Vec::new(),
            feature_pointer: 0,
            loaded: false,
            feature_pointer_published: false,
            writes: 0,
            fail_upload: false,
            corrupt_enable: false,
            corrupt_register: None,
        }
    }
    fn pointer(&self) -> usize {
        ((usize::from(self.registers[0x5c]) << 4) | usize::from(self.registers[0x5b] & 15)) * 2
    }
    fn write(&mut self, bytes: &[u8]) -> Result<(), Fault> {
        assert!(bytes.len() <= 17, "shared EasyDMA buffer requirement");
        self.writes += 1;
        let reg = bytes[0] as usize;
        let data = &bytes[1..];
        if reg == 0x5e {
            if self.fail_upload {
                return Err(Fault);
            }
            let pointer = self.pointer();
            if self.loaded {
                let offset = pointer - self.feature_pointer;
                self.features[offset..offset + data.len()].copy_from_slice(data);
            } else {
                assert_eq!(pointer, self.uploaded.len(), "upload word address");
                self.uploaded.extend_from_slice(data);
            }
        } else {
            self.registers[reg..reg + data.len()].copy_from_slice(data);
            if reg == 0x59 && data == [1] {
                self.loaded = true;
                self.status = self.completion_status;
                self.feature_pointer = 0x406;
            }
            // Exercise the vendor's APS handoff rather than assuming INIT_CTRL
            // immediately replaces the upload address with the feature pointer.
            if reg == 0x7c && data[0] & 1 != 0 && self.loaded && !self.feature_pointer_published {
                self.registers[0x5b] = 3;
                self.registers[0x5c] = 0x20;
                self.feature_pointer_published = true;
            }
        }
        Ok(())
    }
    fn read(&mut self, reg: usize, bytes: &mut [u8]) {
        if reg == 0x00 {
            bytes[0] = self.id;
        } else if reg == 0x2a {
            bytes[0] = self.status;
        } else if reg == 0x5e {
            let offset = self.pointer() - self.feature_pointer;
            bytes.copy_from_slice(&self.features[offset..offset + bytes.len()]);
            if self.corrupt_enable && offset == 0x3a {
                bytes[1] &= !0x10;
            }
        } else {
            bytes.copy_from_slice(&self.registers[reg..reg + bytes.len()]);
            if self.loaded {
                if let Some((register, value)) = self.corrupt_register {
                    if reg == usize::from(register) {
                        bytes[0] = value;
                    }
                }
            }
        }
    }
}

struct Bus(Rc<RefCell<Sensor>>);
impl ErrorType for Bus {
    type Error = Fault;
}
impl I2c for Bus {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Fault> {
        assert_eq!(address, 0x18);
        let mut sensor = self.0.borrow_mut();
        let mut register = 0;
        for op in operations {
            match op {
                Operation::Write(data) if data.len() == 1 => register = data[0] as usize,
                Operation::Write(data) => sensor.write(data)?,
                Operation::Read(data) => sensor.read(register, data),
            }
        }
        Ok(())
    }
}
struct Delay;
impl DelayNs for Delay {
    async fn delay_ns(&mut self, _: u32) {}
}
struct NoWake;
impl Wake for NoWake {
    fn wake(self: Arc<Self>) {}
}
fn run<F: Future>(future: F) -> F::Output {
    let waker = Waker::from(Arc::new(NoWake));
    let mut cx = Context::from_waker(&waker);
    let mut future = std::pin::pin!(future);
    match future.as_mut().poll(&mut cx) {
        Poll::Ready(value) => value,
        Poll::Pending => panic!("fake transport unexpectedly blocked"),
    }
}
fn fixture(id: u8) -> (Motion<Bus, Delay>, Rc<RefCell<Sensor>>) {
    let sensor = Rc::new(RefCell::new(Sensor::new(id)));
    (Motion::new(Bus(sensor.clone()), Delay), sensor)
}

#[test]
fn unknown_sensor_is_never_written_and_uninitialized_counter_is_unavailable() {
    let (mut driver, sensor) = fixture(0x42);
    assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
    assert_eq!(run(driver.initialize()), Err(Error::UnknownChip(0x42)));
    run(driver.stop()).unwrap();
    assert_eq!(sensor.borrow().writes, 0);
}

#[test]
fn both_variants_upload_their_exact_stream_and_count_little_endian() {
    for (id, variant, expected) in [
        (
            0x11,
            Variant::Bma421,
            include_bytes!("../../../src/device/bosch/bma423.bin"),
        ),
        (
            0x13,
            Variant::Bma425,
            include_bytes!("../../../src/device/bosch/bma425.bin"),
        ),
    ] {
        let (mut driver, sensor) = fixture(id);
        let config = run(driver.initialize()).unwrap();
        assert_eq!(config.variant, variant);
        assert_eq!(config.id, 0x1234);
        let mut state = sensor.borrow_mut();
        assert_eq!(&state.uploaded, expected);
        assert_eq!(state.features[0x3b], 0x10);
        assert_eq!(state.registers[0x7c] & 1, 1);
        assert_eq!(state.registers[0x7d] & 4, 4);
        state.registers[0x1e..0x22].copy_from_slice(&0x12345678u32.to_le_bytes());
        drop(state);
        assert_eq!(run(driver.counter()), Ok(0x12345678));
        run(driver.stop()).unwrap();
        assert_eq!(sensor.borrow().registers[0x7d] & 4, 0);
        assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
    }
}

#[test]
fn failed_upload_or_bad_asic_status_never_enables_acceleration() {
    for fail_upload in [true, false] {
        let (mut driver, sensor) = fixture(0x11);
        sensor.borrow_mut().fail_upload = fail_upload;
        sensor.borrow_mut().completion_status = 2;
        let error = run(driver.initialize()).unwrap_err();
        assert_eq!(
            error,
            if fail_upload {
                Error::Bus(Fault)
            } else {
                Error::InitializationStatus(2)
            }
        );
        assert_eq!(sensor.borrow().registers[0x7d] & 4, 0);
        assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
        if fail_upload {
            assert!(sensor.borrow().uploaded.is_empty());
        }
    }
}

#[test]
fn feature_readback_mismatch_is_not_reported_as_ready() {
    let (mut driver, sensor) = fixture(0x13);
    sensor.borrow_mut().corrupt_enable = true;
    assert_eq!(
        run(driver.initialize()),
        Err(Error::FeatureReadback {
            start: 0x203,
            config_id: 0x1234,
            expected: [0, 0x10],
            actual: [0, 0],
        })
    );
    assert_eq!(sensor.borrow().registers[0x7d] & 4, 0);
    assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
}

#[test]
fn each_register_readback_failure_reports_its_address_and_actual_value() {
    for (register, mask, expected, actual) in [
        (0x40, 0xff, 0x28, 0xa8),
        (0x41, 0x03, 0, 1),
        (0x7d, 0x04, 4, 0),
        (0x7c, 0x01, 1, 0),
    ] {
        let (mut driver, sensor) = fixture(0x11);
        sensor.borrow_mut().corrupt_register = Some((register, actual));
        assert_eq!(
            run(driver.initialize()),
            Err(Error::RegisterReadback {
                register,
                mask,
                expected,
                actual,
            })
        );
        assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
        sensor.borrow_mut().corrupt_register = None;
        run(driver.stop()).unwrap();
        assert_eq!(sensor.borrow().registers[0x7d] & 4, 0);
    }
}

#[test]
fn lost_engine_state_makes_counts_unavailable_and_shutdown_restores_power_save() {
    let (mut driver, sensor) = fixture(0x11);
    run(driver.initialize()).unwrap();
    sensor.borrow_mut().status = 0;
    assert_eq!(run(driver.counter()), Err(Error::InitializationStatus(0)));
    assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
    sensor.borrow_mut().registers[0x7c] &= !1;
    run(driver.stop()).unwrap();
    assert_eq!(sensor.borrow().registers[0x7c] & 1, 1);
    assert_eq!(sensor.borrow().registers[0x7d] & 4, 0);
}

#[test]
fn previously_initialized_asic_is_rejected_before_upload_or_sensor_writes() {
    let (mut driver, sensor) = fixture(0x11);
    sensor.borrow_mut().status = 1;
    assert_eq!(run(driver.initialize()), Err(Error::NeedsReset(1)));
    assert_eq!(sensor.borrow().writes, 0);
    assert!(sensor.borrow().uploaded.is_empty());
    assert_eq!(run(driver.counter()), Err(Error::NotInitialized));
}
