# Bosch feature configuration

These unchanged 6144-byte streams were extracted from `bma423_config_file` and
`bma425_config_file` in Bosch Sensortec's BSD-3-Clause `bma423.c` V2.14.13
(2020-05-08), distributed by InfiniTime at commit
`9ad16304f5fd526b781a760cf1313813d232645e`:

https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/drivers/Bma421_C/bma423.c

The matching `bma4.c`, `bma4_defs.h` and `bma423.h` at that revision specify
upload addressing, delays, feature layout and counter registers. See LICENSE;
retain its notice with firmware distributions. The GPL InfiniTime C++ wrapper
is not incorporated into Kongle.

| File | Sensor ID selected by the matching Bosch API | SHA-256 |
| --- | --- | --- |
| bma423.bin | 0x11 (PineTime BMA421) | 989cc14b87803ed3406d3571fbd2832049b6748a5410aaa719c7b5753ca506c3 |
| bma425.bin | 0x13 (PineTime BMA425) | 6f3c2a818a874cc8498728b22fd975518e60733cc2f244f400edd6830aca89a2 |

Extraction: isolate the named C array body and decode each `0xNN` literal, in
order. No headers, padding or edits are added. Unknown chip IDs must be rejected
before any sensor write. Hardware validation is still required; byte-level
provenance does not establish step accuracy or successful initialization.
