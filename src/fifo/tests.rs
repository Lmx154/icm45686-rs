//! Host tests for the FIFO frame decoder (DS-000489 Rev 1.1). Header bits are from 5.2 (p40, p41);
//! the layouts are from 5.1 (p39), read as little endian (14, p57). The encoder below exists only
//! here.

use proptest::prelude::*;

use super::*;
use crate::config::{FifoConfig, FifoFormat, TimestampResolution};

fn cfg(format: FifoFormat, accel: bool, gyro: bool) -> FifoConfig {
    let mut c = FifoConfig::new();
    c.format = format;
    c.accel = accel;
    c.gyro = gyro;
    c
}

fn standard() -> FifoConfig {
    cfg(FifoFormat::Standard, true, true)
}
fn accel_only() -> FifoConfig {
    cfg(FifoFormat::Standard, true, false)
}
fn gyro_only() -> FifoConfig {
    cfg(FifoFormat::Standard, false, true)
}
fn hires() -> FifoConfig {
    cfg(FifoFormat::HighResolution, true, true)
}

fn frames<'a>(buf: &'a [u8], c: &FifoConfig) -> Vec<FifoFrame<'a>> {
    fifo_frames(buf, c).collect()
}

const NO_FLAGS: FifoFlags = FifoFlags {
    timestamp_field: false,
    fsync_tag: false,
    accel_odr_changed: false,
    gyro_odr_changed: false,
};

#[test]
fn header_bits_and_frame_sizes_follow_the_tables() {
    // 5.2 p40: bit 6 ACCEL_EN, bit 5 GYRO_EN, bit 4 HIRES_EN, bit 3 TMST_FIELD_EN, bit 2
    // FSYNC_TAG_EN; p41: bit 1 ACCEL_ODR, bit 0 GYRO_ODR. 5.1 p39: 8, 8, 16, 20 bytes.
    assert_eq!(0x40, 1 << 6);
    assert_eq!(0x20, 1 << 5);
    assert_eq!(0x10, 1 << 4);
    assert_eq!(HEADER_TMST_FIELD, 1 << 3);
    assert_eq!(HEADER_FSYNC_TAG, 1 << 2);
    assert_eq!(HEADER_ACCEL_ODR, 1 << 1);
    assert_eq!(HEADER_GYRO_ODR, 1);
    assert_eq!(HEADER_EXT, 1 << 7);
    for c in [standard(), accel_only(), gyro_only(), hires()] {
        let expected = fifo_frames(&[], &c).size;
        assert_eq!(expected, usize::from(c.frame_bytes()));
    }
}

#[test]
fn golden_accel_only() {
    // 5.1 p39 Accel-only: Header, AxH AxL AyH AyL AzH AzL, Temp. Header ACCEL_EN only (5.2 p40).
    let b = [0x42, 0x34, 0x12, 0xFF, 0xFF, 0x00, 0x80, 0xFE];
    assert_eq!(
        frames(&b, &accel_only()),
        [FifoFrame::Accel(AccelFrame {
            flags: FifoFlags {
                accel_odr_changed: true,
                ..NO_FLAGS
            },
            accel: [0x1234, -1, i16::MIN],
            temp: -2,
        })]
    );
}

#[test]
fn golden_gyro_only() {
    // 5.1 p39 Gyro-only: Header, GxH GxL ... GzL, Temp. Header GYRO_EN only (5.2 p40).
    let b = [0x21, 0x01, 0x00, 0xFF, 0x7F, 0x00, 0x80, 0x32];
    assert_eq!(
        frames(&b, &gyro_only()),
        [FifoFrame::Gyro(GyroFrame {
            flags: FifoFlags {
                gyro_odr_changed: true,
                ..NO_FLAGS
            },
            gyro: [1, i16::MAX, i16::MIN],
            temp: 0x32,
        })]
    );
}

#[test]
fn golden_accel_gyro() {
    // 5.1 p39 Accel+Gyro: Header, accel x3, gyro x3, Temp, TimestampH, TimestampL. Header
    // ACCEL_EN | GYRO_EN | TMST_FIELD_EN | FSYNC_TAG_EN (5.2 p40).
    let b = [
        0x6C, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0xFF, 0xFF, 0xFE, 0xFF, 0xFD, 0xFF, 0x40, 0x34,
        0x12,
    ];
    assert_eq!(
        frames(&b, &standard()),
        [FifoFrame::AccelGyro(AccelGyroFrame {
            flags: FifoFlags {
                timestamp_field: true,
                fsync_tag: true,
                ..NO_FLAGS
            },
            accel: [1, 2, 3],
            gyro: [-1, -2, -3],
            temp: 0x40,
            timestamp: 0x1234,
        })]
    );
}

#[test]
fn golden_high_resolution() {
    // 5.1 p39 High resolution: Header, accel x3 (16 bits), gyro x3 (16 bits), Temp H/L,
    // Timestamp H/L, then one byte per axis with the accel nibble in bits 7:4 and the gyro nibble
    // in bits 3:0. Header ACCEL_EN | GYRO_EN | HIRES_EN | TMST_FIELD_EN (5.2 p40).
    let b = [
        0x78, 0x34, 0x12, 0x00, 0x80, 0xFF, 0xFF, 0x78, 0x56, 0xFF, 0x7F, 0x00, 0x80, 0x00, 0x0A,
        0x10, 0x00, 0xA5, 0x0F, 0xF0,
    ];
    assert_eq!(
        frames(&b, &hires()),
        [FifoFrame::HighRes(HighResFrame {
            flags: FifoFlags {
                timestamp_field: true,
                ..NO_FLAGS
            },
            accel: [0x1234A, i32::from(i16::MIN) * 16, -16 + 0xF],
            gyro: [0x56785, 0x7FFFF, i32::from(i16::MIN) * 16],
            temp: 0x0A00,
            timestamp: 0x0010,
        })]
    );
}

#[test]
fn twenty_bit_sign_extension_at_both_extremes() {
    // 20 bit two's complement: 0x80000 is -524288 (the minimum), 0x7FFFF is +524287 (5, p39).
    let mut b = [0u8; 20];
    b[0] = 0x70;
    // Accel X = 0x80000: word 0x8000, nibble 0. Accel Y = 0x7FFFF: word 0x7FFF, nibble 0xF.
    b[1] = 0x00;
    b[2] = 0x80;
    b[3] = 0xFF;
    b[4] = 0x7F;
    b[17] = 0x00;
    b[18] = 0xF0;
    // Gyro X = 0x80000, gyro Y = 0x7FFFF.
    b[7] = 0x00;
    b[8] = 0x80;
    b[9] = 0xFF;
    b[10] = 0x7F;
    b[18] |= 0x0F;
    let out = frames(&b, &hires());
    let [FifoFrame::HighRes(f)] = out[..] else {
        panic!("{out:?}")
    };
    assert_eq!(f.accel, [-524_288, 524_287, 0]);
    assert_eq!(f.gyro, [-524_288, 524_287, 0]);
}

#[test]
fn invalid_headers_come_out_raw() {
    // 5.2 p40: bit 7 is the extended header (external sensors, compression): out of scope.
    let ext = [0xE0u8; 16];
    assert_eq!(frames(&ext, &standard()), [FifoFrame::Invalid(&ext)]);
    // No sensor bit set, and bit patterns that are none of the four layouts of 5.1.
    for h in [0x00u8, 0x10, 0x30, 0x50] {
        let b = [h; 16];
        assert_eq!(frames(&b, &standard()), [FifoFrame::Invalid(&b)], "{h:#x}");
    }
    // An 8-byte layout cannot carry a timestamp or an FSYNC tag (5.2 p40, TMST_FIELD_EN).
    for h in [0x48u8, 0x44, 0x28, 0x24] {
        let b = [h; 8];
        assert_eq!(
            frames(
                &b,
                &if h & 0x40 != 0 {
                    accel_only()
                } else {
                    gyro_only()
                }
            ),
            [FifoFrame::Invalid(&b)],
            "{h:#x}"
        );
    }
    // No sensor configured: nothing can be decoded, all bytes in one item.
    let none = cfg(FifoFormat::Standard, false, false);
    let b = [0x60u8; 5];
    assert_eq!(frames(&b, &none), [FifoFrame::Invalid(&b)]);
}

#[test]
fn wrong_format_header() {
    // Accel+gyro header (5.2 p40) while accel only is configured, and the reverse.
    let b = [0x60u8; 8];
    assert_eq!(frames(&b, &accel_only()), [FifoFrame::WrongFormat(&b)]);
    let b = [0x40u8; 16];
    assert_eq!(frames(&b, &standard()), [FifoFrame::WrongFormat(&b)]);
    let b = [0x60u8; 20];
    assert_eq!(frames(&b, &hires()), [FifoFrame::WrongFormat(&b)]);
    let b = [0x70u8; 16];
    assert_eq!(frames(&b, &standard()), [FifoFrame::WrongFormat(&b)]);
}

#[test]
fn trailing_bytes() {
    let mut b = vec![0x60u8; 16];
    b.extend_from_slice(&[0x60, 1, 2]);
    let out = frames(&b, &standard());
    assert_eq!(out.len(), 2);
    assert!(matches!(out[0], FifoFrame::AccelGyro(_)));
    assert_eq!(out[1], FifoFrame::Trailing(&[0x60, 1, 2]));
    assert_eq!(
        frames(&[0x40], &accel_only()),
        [FifoFrame::Trailing(&[0x40])]
    );
    assert!(frames(&[], &standard()).is_empty());
}

#[test]
fn the_invalid_sample_marker_is_reported_and_still_decodes_as_a_number() {
    // Vendor (TDK's Linux driver): an axis value of 0x8000 marks a sample the device could not
    // provide. The datasheet defines no marker, so the decoder keeps returning -32768 as the number
    // it is; the predicate reports the marker. The 20-bit format has no marker (none is documented).
    assert_eq!(INVALID_SAMPLE_I16, -32768);
    let b = [0x40, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00];
    let out = frames(&b, &accel_only());
    assert!(matches!(
        out[..],
        [FifoFrame::Accel(AccelFrame {
            accel: [i16::MIN, i16::MIN, i16::MIN],
            ..
        })]
    ));
    let [FifoFrame::Accel(a)] = out[..] else {
        panic!("not an accel frame");
    };
    assert!(a.accel_invalid());

    let g = GyroFrame {
        flags: FifoFlags::default(),
        gyro: [i16::MIN, i16::MIN, i16::MIN],
        temp: 0,
    };
    assert!(g.gyro_invalid());
}

#[test]
fn the_invalid_sample_marker_needs_all_three_axes() {
    let m = INVALID_SAMPLE_I16;
    let mut a = AccelFrame {
        flags: FifoFlags::default(),
        accel: [m, m, m + 1],
        temp: 0,
    };
    assert!(!a.accel_invalid());
    a.accel = [0, 0, 0];
    assert!(!a.accel_invalid());
    let mut g = GyroFrame {
        flags: FifoFlags::default(),
        gyro: [m, 0, m],
        temp: 0,
    };
    assert!(!g.gyro_invalid());
    g.gyro = [i16::MAX, m, m];
    assert!(!g.gyro_invalid());

    // Both sensors in one frame are judged separately.
    let ag = AccelGyroFrame {
        flags: FifoFlags::default(),
        accel: [m, m, m],
        gyro: [1, 2, 3],
        temp: 0,
        timestamp: 0,
    };
    assert!(ag.accel_invalid());
    assert!(!ag.gyro_invalid());
    let swapped = AccelGyroFrame {
        accel: ag.gyro,
        gyro: ag.accel,
        ..ag
    };
    assert!(!swapped.accel_invalid());
    assert!(swapped.gyro_invalid());
}

#[test]
fn conversions() {
    // 16.14 p69: 8-bit FIFO temperature = raw / 2 + 25; 16-bit = raw / 128 + 25.
    assert_eq!(temperature_c_8bit(0), 25.0);
    assert_eq!(temperature_c_8bit(10), 30.0);
    assert_eq!(temperature_c_8bit(-80), -15.0);
    assert_eq!(temperature_c_16bit(0), 25.0);
    assert_eq!(temperature_c_16bit(1280), 35.0);
    assert_eq!(temperature_c_16bit(-3200), 0.0);
    // 5 p39: 16384 LSB/g and 131.1 LSB/dps (4000 dps over 2^19).
    let a = hires_accel_ms2([16384, -16384, 524_287]);
    assert!((a[0] - 9.80665).abs() < 1e-6 && (a[1] + 9.80665).abs() < 1e-6);
    assert!((a[2] - 32.0 * 9.80665).abs() < 1e-3);
    let g = hires_gyro_rads([524_288, -524_288, 0]);
    let full = 4000.0_f32.to_radians();
    assert!((g[0] - full).abs() < 1e-2 && (g[1] + full).abs() < 1e-2 && g[2] == 0.0);
    assert!((HIRES_GYRO_LSB_PER_DPS - 131.1).abs() < 0.05);
    // 16.34 p84: TMST_RESOL 0 is 1 us, 1 is 16 us.
    assert_eq!(timestamp_us(1000, TimestampResolution::Us1), 1000);
    assert_eq!(timestamp_us(1000, TimestampResolution::Us16), 16_000);
    assert_eq!(timestamp_us(u16::MAX, TimestampResolution::Us16), 1_048_560);
    assert_eq!(timestamp_us(0, TimestampResolution::Us16), 0);
}

fn flags_byte(f: &FifoFlags) -> u8 {
    u8::from(f.timestamp_field) << 3
        | u8::from(f.fsync_tag) << 2
        | u8::from(f.accel_odr_changed) << 1
        | u8::from(f.gyro_odr_changed)
}

fn put_word(out: &mut Vec<u8>, v: i16) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn encode(frame: &FifoFrame<'_>) -> Vec<u8> {
    let mut o = Vec::new();
    match frame {
        FifoFrame::Accel(f) => {
            o.push(0x40 | flags_byte(&f.flags));
            f.accel.iter().for_each(|&v| put_word(&mut o, v));
            o.push(f.temp as u8);
        }
        FifoFrame::Gyro(f) => {
            o.push(0x20 | flags_byte(&f.flags));
            f.gyro.iter().for_each(|&v| put_word(&mut o, v));
            o.push(f.temp as u8);
        }
        FifoFrame::AccelGyro(f) => {
            o.push(0x60 | flags_byte(&f.flags));
            f.accel.iter().for_each(|&v| put_word(&mut o, v));
            f.gyro.iter().for_each(|&v| put_word(&mut o, v));
            o.push(f.temp as u8);
            o.extend_from_slice(&f.timestamp.to_le_bytes());
        }
        FifoFrame::HighRes(f) => {
            o.push(0x70 | flags_byte(&f.flags));
            f.accel
                .iter()
                .for_each(|&v| put_word(&mut o, (v >> 4) as i16));
            f.gyro
                .iter()
                .for_each(|&v| put_word(&mut o, (v >> 4) as i16));
            put_word(&mut o, f.temp);
            o.extend_from_slice(&f.timestamp.to_le_bytes());
            for i in 0..3 {
                o.push((((f.accel[i] & 0xF) as u8) << 4) | (f.gyro[i] & 0xF) as u8);
            }
        }
        FifoFrame::Invalid(b) | FifoFrame::WrongFormat(b) | FifoFrame::Trailing(b) => {
            o.extend_from_slice(b);
        }
    }
    o
}

fn flags() -> impl Strategy<Value = FifoFlags> {
    (any::<bool>(), any::<bool>(), any::<bool>(), any::<bool>()).prop_map(|(t, s, a, g)| {
        FifoFlags {
            timestamp_field: t,
            fsync_tag: s,
            accel_odr_changed: a,
            gyro_odr_changed: g,
        }
    })
}

fn single_flags() -> impl Strategy<Value = FifoFlags> {
    (any::<bool>(), any::<bool>()).prop_map(|(a, g)| FifoFlags {
        accel_odr_changed: a,
        gyro_odr_changed: g,
        ..NO_FLAGS
    })
}

fn wide20() -> impl Strategy<Value = i32> {
    -524_288i32..=524_287
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 600, ..ProptestConfig::default() })]

    #[test]
    fn arbitrary_bytes_never_panic_and_are_all_accounted_for(
        buf in proptest::collection::vec(any::<u8>(), 0..300),
        hires_on in any::<bool>(),
        accel in any::<bool>(),
        gyro in any::<bool>(),
    ) {
        let format = if hires_on { FifoFormat::HighResolution } else { FifoFormat::Standard };
        let c = cfg(format, accel, gyro);
        let mut total = 0;
        let mut joined = Vec::new();
        for f in fifo_frames(&buf, &c) {
            total += f.size_bytes();
            if let FifoFrame::Invalid(b) | FifoFrame::WrongFormat(b) | FifoFrame::Trailing(b) = f {
                prop_assert!(!b.is_empty());
                joined.extend_from_slice(b);
            }
        }
        prop_assert_eq!(total, buf.len());
        // Raw items are slices of the input, in order.
        let mut it = buf.as_slice();
        for b in fifo_frames(&buf, &c) {
            let n = b.size_bytes();
            if let FifoFrame::Invalid(raw) | FifoFrame::WrongFormat(raw) | FifoFrame::Trailing(raw) = b {
                prop_assert_eq!(raw, it.get(..n).unwrap_or(&[]));
            }
            it = it.get(n..).unwrap_or(&[]);
        }
        prop_assert!(it.is_empty());
    }

    #[test]
    fn encoder_decoder_round_trip(
        a in proptest::collection::vec((single_flags(), any::<[i16; 3]>(), any::<i8>()), 0..4),
        g in proptest::collection::vec((single_flags(), any::<[i16; 3]>(), any::<i8>()), 0..4),
        ag in proptest::collection::vec(
            (flags(), any::<[i16; 3]>(), any::<[i16; 3]>(), any::<i8>(), any::<u16>()), 0..4),
        h in proptest::collection::vec(
            (flags(), [wide20(), wide20(), wide20()], [wide20(), wide20(), wide20()],
             any::<i16>(), any::<u16>()), 0..4),
    ) {
        let want_a: Vec<_> = a.iter().map(|&(flags, accel, temp)|
            FifoFrame::Accel(AccelFrame { flags, accel, temp })).collect();
        let want_g: Vec<_> = g.iter().map(|&(flags, gyro, temp)|
            FifoFrame::Gyro(GyroFrame { flags, gyro, temp })).collect();
        let want_ag: Vec<_> = ag.iter().map(|&(flags, accel, gyro, temp, timestamp)|
            FifoFrame::AccelGyro(AccelGyroFrame { flags, accel, gyro, temp, timestamp })).collect();
        let want_h: Vec<_> = h.iter().map(|&(flags, accel, gyro, temp, timestamp)|
            FifoFrame::HighRes(HighResFrame { flags, accel, gyro, temp, timestamp })).collect();
        for (want, c) in [
            (want_a, accel_only()), (want_g, gyro_only()), (want_ag, standard()), (want_h, hires()),
        ] {
            let bytes: Vec<u8> = want.iter().flat_map(encode).collect();
            prop_assert_eq!(frames(&bytes, &c), want);
        }
    }

    #[test]
    fn conversions_stay_in_range(t8 in any::<i8>(), t16 in any::<i16>(), ticks in any::<u16>(),
                                 raw in [wide20(), wide20(), wide20()]) {
        prop_assert!((-103.0..=89.0).contains(&temperature_c_8bit(t8)));
        prop_assert!((-231.0..=281.0).contains(&temperature_c_16bit(t16)));
        prop_assert_eq!(timestamp_us(ticks, TimestampResolution::Us16), u32::from(ticks) * 16);
        prop_assert!(hires_accel_ms2(raw).iter().all(|v| v.abs() <= 32.0 * 9.80665 + 1e-3));
        prop_assert!(hires_gyro_rads(raw).iter().all(|v| v.abs() <= 4000.0_f32.to_radians() + 1e-2));
    }
}

/// The fast path of `FifoFrames::next` and the slow `decode` give the same item for every header
/// byte, in every frame layout, with the same payload.
#[test]
fn fast_path_agrees_with_decode_for_every_header() {
    for config in [accel_only(), gyro_only(), standard(), hires()] {
        let probe = fifo_frames(&[], &config);
        let (expected, size) = (probe.expected, probe.size);
        for header in 0..=u8::MAX {
            let mut frame = [0u8; FRAME_BYTES_HIRES];
            for (i, byte) in frame.iter_mut().enumerate() {
                *byte = (i as u8).wrapping_mul(0x37).wrapping_add(0x5B);
            }
            frame[0] = header;
            let frame = &frame[..size];
            assert_eq!(
                fifo_frames(frame, &config).next(),
                Some(decode(frame, expected)),
                "header {header:#04x}, frame size {size}"
            );
        }
    }
}

fn layouts() -> [FifoConfig; 4] {
    [accel_only(), gyro_only(), standard(), hires()]
}

/// Writes the invalid-sample marker 0x8000 (little endian) into the 16-bit words of `payload`
/// (bytes 1 to 12) that `mask` selects; `mode` 0 and 1 leave the payload alone, 2 uses `mask`,
/// 3 sets every word.
fn mark_invalid(payload: &mut [u8], mode: u8, mask: u8) {
    for k in 0..6usize {
        let selected = match mode {
            2 => mask & (1 << k) != 0,
            3 => true,
            _ => false,
        };
        if selected && let Some(word) = payload.get_mut(1 + 2 * k..3 + 2 * k) {
            word.copy_from_slice(&[0x00, 0x80]);
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 2000, ..ProptestConfig::default() })]

    /// One frame of any layout, any header, any payload (with the 0x8000 marker in some or all
    /// words with noticeable probability): the iterator's item equals the slow `decode`.
    #[test]
    fn fast_path_agrees_with_decode_on_arbitrary_frames(
        layout in 0usize..4,
        header in any::<u8>(),
        bias in any::<bool>(),
        raw in proptest::collection::vec(any::<u8>(), FRAME_BYTES_HIRES),
        mode in 0u8..4,
        mask in any::<u8>(),
    ) {
        let config = layouts()[layout];
        let probe = fifo_frames(&[], &config);
        let (expected, size) = (probe.expected, probe.size);
        let mut frame = raw;
        frame.truncate(size);
        mark_invalid(&mut frame, mode, mask);
        frame[0] = if bias { (header & 0x0F) | (expected << KIND_SHIFT) } else { header };
        prop_assert_eq!(
            fifo_frames(&frame, &config).next(),
            Some(decode(&frame, expected))
        );
    }

    /// A whole buffer: 1 to 40 frames of one layout plus 0 to size - 1 trailing bytes. Every item
    /// equals `decode` of its chunk, and the trailing bytes come out as one `Trailing` item.
    #[test]
    fn iterator_agrees_with_decode_on_whole_buffers(
        layout in 0usize..4,
        frames in 1usize..=40,
        tail in 0usize..FRAME_BYTES_HIRES,
        raw in proptest::collection::vec(any::<u8>(), 40 * FRAME_BYTES_HIRES + FRAME_BYTES_HIRES),
        heads in proptest::collection::vec((any::<u8>(), any::<bool>(), 0u8..4, any::<u8>()), 40),
    ) {
        let config = layouts()[layout];
        let probe = fifo_frames(&[], &config);
        let (expected, size) = (probe.expected, probe.size);
        let tail = tail % size;
        let mut buf = raw;
        buf.truncate(frames * size + tail);
        for (i, (header, bias, mode, mask)) in heads.iter().take(frames).enumerate() {
            let chunk = &mut buf[i * size..(i + 1) * size];
            mark_invalid(chunk, *mode, *mask);
            chunk[0] = if *bias { (header & 0x0F) | (expected << KIND_SHIFT) } else { *header };
        }
        let items: Vec<_> = fifo_frames(&buf, &config).collect();
        prop_assert_eq!(items.len(), frames + usize::from(tail > 0));
        for (i, chunk) in buf.chunks_exact(size).enumerate() {
            prop_assert_eq!(items[i], decode(chunk, expected));
        }
        if tail > 0 {
            prop_assert_eq!(items[frames], FifoFrame::Trailing(&buf[frames * size..]));
        }
    }
}
