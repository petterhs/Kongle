//! Replay numeric CSV from scripts/hr-diagnostics-to-csv.py through the actual
//! firmware estimator. Small scheduling jitter is resampled; real gaps reset.
#[allow(dead_code)]
#[path = "../../../src/heart_rate.rs"]
mod heart_rate;

use std::{error::Error, fs, io};

fn main() -> Result<(), Box<dyn Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: replay-heart-rate trace.csv")?;
    let data = fs::read_to_string(path)?;
    let mut lines = data.lines();
    let header: Vec<_> = lines.next().ok_or("empty CSV")?.split(',').collect();
    let column = |name: &str| {
        header
            .iter()
            .position(|field| *field == name)
            .ok_or_else(|| format!("missing column: {name}"))
    };
    let columns = [
        column("session")?,
        column("sample")?,
        column("elapsed_ms")?,
        column("gain")?,
        column("ppg")?,
        column("ambient")?,
    ];
    let mut estimator = heart_rate::Estimator::new();
    let mut current = None;
    let mut samples = 0;
    let mut readings = 0;
    println!("session,sample,elapsed_ms,replayed_bpm,reason,filled,timing_resets,analyses,concentration,similarity");
    for line in lines.filter(|line| !line.trim().is_empty()) {
        let fields: Vec<_> = line.split(',').collect();
        let value = |index: usize| -> Result<u64, Box<dyn Error>> {
            let text = fields.get(columns[index]).ok_or("truncated CSV row")?;
            Ok(text.parse()?)
        };
        let session = value(0)?;
        let count = value(1)?;
        let elapsed = value(2)?;
        let gain = value(3)?;
        let ppg = u32::try_from(value(4)?)?;
        let ambient = u32::try_from(value(5)?)?;
        if !matches!(gain, 1 | 2 | 4 | 8) {
            return Err(io::Error::other("invalid gain").into());
        }
        if current != Some((session, gain)) {
            estimator = heart_rate::Estimator::new();
            current = Some((session, gain));
        }
        samples += 1;
        let result = estimator.sample(ppg, ambient, elapsed);
        if result.is_some() {
            readings += 1;
        }
        let quality = estimator.quality();
        let bpm = result.map(|bpm| bpm.to_string()).unwrap_or_default();
        println!(
            "{session},{count},{elapsed},{bpm},{:?},{},{},{},{},{}",
            quality.reason,
            quality.filled,
            quality.timing_resets,
            quality.analyses,
            quality.concentration,
            quality.similarity
        );
    }
    eprintln!("Replayed {samples} samples, {readings} qualified readings; expected cadence {} ms. Gaps reset history.", heart_rate::SAMPLE_MS);
    Ok(())
}
