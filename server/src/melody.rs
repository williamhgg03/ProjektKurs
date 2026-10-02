// Melody for the piezo buzzer. An uploaded audio file is decoded to mono, the
// strongest pitch is tracked frame by frame and the result is cleaned up into
// a list of notes the firmware plays as square-wave tones. Picking "the melody"
// out of a full mix is a best-effort guess: songs with a clear lead or vocal
// come out well, busy tracks only approximately.

use std::fmt;
use std::io::Cursor;

use rustfft::FftPlanner;
use rustfft::num_complex::Complex;
use serde::Serialize;
use symphonia::core::codecs::audio::AudioDecoderOptions;
use symphonia::core::errors::Error as SymphoniaError;
use symphonia::core::formats::probe::Hint;
use symphonia::core::formats::{FormatOptions, TrackType};
use symphonia::core::io::MediaSourceStream;
use symphonia::core::meta::MetadataOptions;

/// Upper bound on song length in notes, keeps the chip's RAM use sane.
pub const MAX_NOTES: usize = 3000;
/// Audio past this point is ignored, keeps decode time and memory bounded.
const MAX_SECONDS: usize = 10 * 60;
pub const MAX_OCTAVE: u8 = 3;

/// Every output note fits in this range. The firmware's 10-bit LEDC timer can
/// reach about 76 Hz - 78 kHz, and piezos are loudest around 2-4 kHz.
pub const MIN_FREQ: f32 = 100.0;
pub const MAX_FREQ: f32 = 4000.0;

/// Analysis step, also the smallest unit of note length.
const HOP_MS: u32 = 50;
/// Analysis window, about 93ms gives ~11 Hz spectral resolution.
const WINDOW_SECS: f32 = 0.093;
/// Decoded audio is box-averaged down to at least this rate. Plenty for the
/// harmonics of C6 and it cuts memory and FFT work.
const ANALYSIS_RATE: u32 = 16_000;
/// Frames quieter than this fraction of the song's loud level are rests.
const SILENCE_RATIO: f32 = 0.1;
/// Notes shorter than this are folded into the previous note.
const MIN_NOTE_MS: u32 = 100;
/// A frame this much louder than the quieter of the two before it starts a
/// new note, so repeated notes of the same pitch aren't merged into one long
/// tone. Two frames because a short dip often straddles a frame boundary.
const ONSET_RATIO: f32 = 1.4;
/// Silence carved out between repeated notes so they are heard as two.
const GAP_MS: u32 = 30;
/// Candidate pitches, C3..=C6 covers most vocals and lead instruments.
const LOWEST_MIDI: u8 = 48;
const HIGHEST_MIDI: u8 = 84;
const HARMONICS: usize = 6;
const HARMONIC_DECAY: f32 = 0.8;

/// One tone (or rest when `freq == 0`). Same shape as `Note` in the firmware's buzzer.rs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Note {
    pub freq: u16,
    pub ms: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Transcription {
    pub notes: Vec<Note>,
    /// The song was cut to MAX_SECONDS or MAX_NOTES.
    pub truncated: bool,
}

#[derive(Debug, PartialEq, Eq)]
pub enum MelodyError {
    InvalidOctave(u8),
    Decode(String),
    NoMelody,
    NothingLoaded,
}

impl fmt::Display for MelodyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MelodyError::InvalidOctave(o) => {
                write!(f, "octave shift must be between 0 and {MAX_OCTAVE}, got {o}")
            }
            MelodyError::Decode(e) => write!(f, "could not decode audio: {e}"),
            MelodyError::NoMelody => write!(f, "no pitched audio found in the file"),
            MelodyError::NothingLoaded => write!(f, "no melody uploaded yet"),
        }
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct MelodyState {
    /// File name of the uploaded song, if the client sent one.
    pub name: Option<String>,
    pub notes: Vec<Note>,
    pub truncated: bool,
    pub playing: bool,
    /// Bumped on every change so clients can detect updates.
    pub version: u64,
}

impl MelodyState {
    /// Replace the song and start playing it.
    pub fn load(&mut self, name: Option<String>, t: Transcription) {
        self.name = name;
        self.notes = t.notes;
        self.truncated = t.truncated;
        self.playing = true;
        self.version += 1;
    }

    /// Play from the beginning, also when already playing.
    pub fn play(&mut self) -> Result<(), MelodyError> {
        if self.notes.is_empty() {
            return Err(MelodyError::NothingLoaded);
        }
        self.playing = true;
        self.version += 1;
        Ok(())
    }

    pub fn stop(&mut self) {
        self.playing = false;
        self.version += 1;
    }
}

/// Decode an audio file (MP3, WAV, OGG, FLAC) and turn it into notes,
/// shifted up by `octave` octaves.
pub fn transcribe<B>(data: B, octave: u8) -> Result<Transcription, MelodyError>
where
    B: AsRef<[u8]> + Send + Sync + 'static,
{
    if octave > MAX_OCTAVE {
        return Err(MelodyError::InvalidOctave(octave));
    }
    let (samples, rate, cut) = decode(data)?;
    let track = pitch_track(&samples, rate);
    let mut notes = to_notes(&track, octave);
    if notes.is_empty() {
        return Err(MelodyError::NoMelody);
    }
    let truncated = cut || notes.len() > MAX_NOTES;
    notes.truncate(MAX_NOTES);
    Ok(Transcription { notes, truncated })
}

/// Mono samples at a reduced rate, plus whether the audio was cut at MAX_SECONDS.
fn decode<B>(data: B) -> Result<(Vec<f32>, u32, bool), MelodyError>
where
    B: AsRef<[u8]> + Send + Sync + 'static,
{
    let err = |e: SymphoniaError| MelodyError::Decode(e.to_string());

    let mss = MediaSourceStream::new(Box::new(Cursor::new(data)), Default::default());
    let mut format = symphonia::default::get_probe()
        .probe(&Hint::new(), mss, FormatOptions::default(), MetadataOptions::default())
        .map_err(err)?;
    let track = format
        .default_track(TrackType::Audio)
        .ok_or_else(|| MelodyError::Decode("no audio track".into()))?;
    let params = track
        .codec_params
        .as_ref()
        .and_then(|p| p.audio())
        .ok_or_else(|| MelodyError::Decode("missing codec parameters".into()))?;
    let mut decoder = symphonia::default::get_codecs()
        .make_audio_decoder(params, &AudioDecoderOptions::default())
        .map_err(err)?;
    let track_id = track.id;

    let mut out = Vec::new();
    let mut interleaved: Vec<f32> = Vec::new();
    // Box-average `step` mono samples into one output sample
    let mut step = 0;
    let mut rate = 0;
    let (mut acc, mut acc_len) = (0.0f32, 0);
    let mut limit = usize::MAX;

    loop {
        let packet = match format.next_packet() {
            Ok(Some(p)) => p,
            Ok(None) => break,
            // A broken stream mid-file still leaves usable audio before it
            Err(e) if !out.is_empty() => {
                tracing::warn!("stopped decoding early: {e}");
                break;
            }
            Err(e) => return Err(err(e)),
        };
        if packet.track_id != track_id {
            continue;
        }
        let buf = match decoder.decode(&packet) {
            Ok(buf) => buf,
            Err(SymphoniaError::DecodeError(_) | SymphoniaError::IoError(_)) => continue,
            Err(e) => return Err(err(e)),
        };
        if step == 0 {
            let src_rate = buf.spec().rate();
            step = (src_rate / ANALYSIS_RATE).max(1) as usize;
            rate = src_rate / step as u32;
            limit = MAX_SECONDS * rate as usize;
        }

        let channels = buf.spec().channels().count().max(1);
        interleaved.resize(buf.samples_interleaved(), 0.0);
        buf.copy_to_slice_interleaved(&mut interleaved);
        for frame in interleaved.chunks_exact(channels) {
            acc += frame.iter().sum::<f32>() / channels as f32;
            acc_len += 1;
            if acc_len == step {
                out.push(acc / step as f32);
                (acc, acc_len) = (0.0, 0);
            }
        }
        if out.len() >= limit {
            out.truncate(limit);
            return Ok((out, rate, true));
        }
    }

    if out.is_empty() {
        return Err(MelodyError::Decode("file contains no audio".into()));
    }
    Ok((out, rate, false))
}

fn midi_to_hz(m: f32) -> f32 {
    440.0 * 2f32.powf((m - 69.0) / 12.0)
}

/// Analysis result for one HOP_MS slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Frame {
    /// Strongest pitch as a MIDI note, `None` where it's quiet.
    pitch: Option<u8>,
    /// Sudden rise in loudness, likely a new note being struck.
    onset: bool,
}

fn pitch_track(samples: &[f32], rate: u32) -> Vec<Frame> {
    let hop = (rate * HOP_MS / 1000) as usize;
    let window = (rate as f32 * WINDOW_SECS) as usize;
    let frames = samples.len().div_ceil(hop);
    let bin_hz = rate as f32 / window as f32;
    let nyquist = rate as f32 / 2.0;

    let hann: Vec<f32> = (0..window)
        .map(|i| 0.5 - 0.5 * (std::f32::consts::TAU * i as f32 / window as f32).cos())
        .collect();
    let fft = FftPlanner::new().plan_fft_forward(window);
    let mut buf = vec![Complex::default(); window];
    let mut mags = vec![0.0f32; window / 2];

    // Window centred on the middle of each hop, zero-padded at the edges
    let frame_at = |i: usize, buf: &mut [Complex<f32>]| {
        let start = (i * hop + hop / 2) as isize - (window / 2) as isize;
        for (j, c) in buf.iter_mut().enumerate() {
            let s = usize::try_from(start + j as isize)
                .ok()
                .and_then(|k| samples.get(k))
                .copied()
                .unwrap_or(0.0);
            *c = Complex::new(s * hann[j], 0.0);
        }
    };

    let rms: Vec<f32> = (0..frames)
        .map(|i| {
            let slice = &samples[i * hop..((i + 1) * hop).min(samples.len())];
            (slice.iter().map(|s| s * s).sum::<f32>() / slice.len() as f32).sqrt()
        })
        .collect();
    let mut sorted = rms.clone();
    sorted.sort_by(f32::total_cmp);
    let loud = sorted.get(sorted.len() * 95 / 100).copied().unwrap_or(0.0);
    let threshold = loud * SILENCE_RATIO;

    // Linear interpolation between FFT bins at an exact frequency
    let mag_at = |mags: &[f32], f: f32| {
        let x = f / bin_hz;
        let i = x as usize;
        let t = x - i as f32;
        let a = mags.get(i).copied().unwrap_or(0.0);
        let b = mags.get(i + 1).copied().unwrap_or(0.0);
        a + (b - a) * t
    };

    // The frame after an onset is often louder still, don't count it twice
    let mut prev_onset = false;
    (0..frames)
        .map(|i| {
            if rms[i] <= threshold {
                prev_onset = false;
                return Frame { pitch: None, onset: false };
            }
            let dip = rms[i.saturating_sub(2)..i].iter().copied().fold(f32::INFINITY, f32::min);
            let onset = !prev_onset && rms[i] > dip * ONSET_RATIO;
            prev_onset = onset;
            frame_at(i, &mut buf);
            fft.process(&mut buf);
            for (m, c) in mags.iter_mut().zip(&buf) {
                *m = c.norm();
            }

            // Harmonic sum: a note's salience is its decaying-weight harmonics' energy
            let salience = |midi: u8| {
                let f0 = midi_to_hz(midi as f32);
                let mut weight = 1.0;
                let mut sum = 0.0;
                for h in 1..=HARMONICS {
                    let f = f0 * h as f32;
                    if f >= nyquist {
                        break;
                    }
                    sum += weight * mag_at(&mags, f);
                    weight *= HARMONIC_DECAY;
                }
                sum
            };
            let pitch =
                (LOWEST_MIDI..=HIGHEST_MIDI).max_by(|&a, &b| salience(a).total_cmp(&salience(b)));
            Frame { pitch, onset }
        })
        .collect()
}

/// 3-frame median of the pitch: removes single-frame blips. Rests win when
/// they are the majority.
fn smooth(track: &[Frame]) -> Vec<Option<u8>> {
    (0..track.len())
        .map(|i| {
            let lo = i.saturating_sub(1);
            let hi = (i + 2).min(track.len());
            let window = &track[lo..hi];
            let mut pitched: Vec<u8> = window.iter().filter_map(|f| f.pitch).collect();
            if pitched.len() * 2 <= window.len() {
                return None;
            }
            pitched.sort_unstable();
            Some(pitched[pitched.len() / 2])
        })
        .collect()
}

/// Shift by octaves until the frequency is in MIN_FREQ..=MAX_FREQ.
fn fold(mut f: f32) -> u16 {
    while f > MAX_FREQ {
        f /= 2.0;
    }
    while f < MIN_FREQ {
        f *= 2.0;
    }
    f.round() as u16
}

/// A run of frames played as one note.
struct Segment {
    pitch: Option<u8>,
    frames: u32,
    /// Starts on an onset, so it stays separate from an equal-pitch predecessor.
    onset: bool,
}

fn to_notes(track: &[Frame], octave: u8) -> Vec<Note> {
    // Split at pitch changes and at onsets
    let mut segs: Vec<Segment> = Vec::new();
    for (pitch, frame) in smooth(track).into_iter().zip(track) {
        let onset = frame.onset && pitch.is_some();
        match segs.last_mut() {
            Some(last) if last.pitch == pitch && !onset => last.frames += 1,
            _ => segs.push(Segment { pitch, frames: 1, onset }),
        }
    }

    // Fold segments too short to hear as notes into their predecessor, which
    // can leave equal-pitch neighbours that only belong apart after an onset
    let mut merged: Vec<Segment> = Vec::with_capacity(segs.len());
    for seg in segs {
        match merged.last_mut() {
            Some(last) if seg.frames * HOP_MS < MIN_NOTE_MS => last.frames += seg.frames,
            Some(last) if last.pitch == seg.pitch && !seg.onset => last.frames += seg.frames,
            _ => merged.push(seg),
        }
    }

    // Leading and trailing silence is just dead air on the buzzer
    while merged.last().is_some_and(|s| s.pitch.is_none()) {
        merged.pop();
    }
    let start = merged.iter().position(|s| s.pitch.is_some()).unwrap_or(merged.len());
    merged.drain(..start);

    let mut notes = Vec::with_capacity(merged.len());
    for (i, seg) in merged.iter().enumerate() {
        let freq = seg.pitch.map_or(0, |m| fold(midi_to_hz((m + 12 * octave) as f32)));
        let ms = seg.frames * HOP_MS;
        let repeated = seg.pitch.is_some() && merged.get(i + 1).is_some_and(|n| n.pitch == seg.pitch);
        if repeated {
            notes.push(Note { freq, ms: ms - GAP_MS });
            notes.push(Note { freq: 0, ms: GAP_MS });
        } else {
            notes.push(Note { freq, ms });
        }
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: u32 = 16_000;
    const A4: u16 = 440;
    const C5: u16 = 523;

    /// Concatenated sine tones, `(freq, ms)` with freq 0 meaning silence.
    fn synth(parts: &[(f32, u32)]) -> Vec<f32> {
        parts
            .iter()
            .flat_map(|&(f, ms)| {
                let n = (RATE * ms / 1000) as usize;
                (0..n).map(move |i| {
                    (std::f32::consts::TAU * f * i as f32 / RATE as f32).sin() * 0.5
                })
            })
            .collect()
    }

    fn notes(parts: &[(f32, u32)], octave: u8) -> Vec<Note> {
        to_notes(&pitch_track(&synth(parts), RATE), octave)
    }

    fn frames(pitches: &[Option<u8>]) -> Vec<Frame> {
        pitches.iter().map(|&pitch| Frame { pitch, onset: false }).collect()
    }

    #[test]
    fn tracks_tones_and_rests() {
        let n = notes(&[(440.0, 500), (0.0, 300), (523.25, 500)], 0);
        let freqs: Vec<u16> = n.iter().map(|n| n.freq).collect();
        assert_eq!(freqs, vec![A4, 0, C5]);
        for (note, want) in n.iter().zip([500, 300, 500]) {
            assert!(note.ms.abs_diff(want) <= HOP_MS, "{note:?}, want {want}ms");
        }
    }

    #[test]
    fn octave_shift_doubles_frequency() {
        let n = notes(&[(440.0, 500)], 1);
        assert_eq!(n, vec![Note { freq: 880, ms: 500 }]);
    }

    #[test]
    fn folds_into_buzzer_range() {
        assert_eq!(fold(midi_to_hz(96.0)), 2093); // C7 already fits
        assert_eq!(fold(8372.0), 2093); // C9 -> C7
        assert_eq!(fold(50.0), 100);
    }

    #[test]
    fn merges_short_blips() {
        let mut track = vec![Some(69); 10];
        track[4] = Some(72);
        track[7] = None;
        assert_eq!(to_notes(&frames(&track), 0), vec![Note { freq: A4, ms: 500 }]);
    }

    #[test]
    fn keeps_repeated_notes_apart() {
        let n = notes(&[(440.0, 400), (0.0, 50), (440.0, 400)], 0);
        let freqs: Vec<u16> = n.iter().map(|n| n.freq).collect();
        assert_eq!(freqs, vec![A4, 0, A4]);
        assert_eq!(n[1].ms, GAP_MS);
    }

    #[test]
    fn trims_leading_and_trailing_silence() {
        let track = [vec![None; 4], vec![Some(69); 4], vec![None; 4]].concat();
        assert_eq!(to_notes(&frames(&track), 0), vec![Note { freq: A4, ms: 200 }]);
    }

    #[test]
    fn silence_has_no_melody() {
        let wav = wav_bytes(&synth(&[(0.0, 500)]));
        assert_eq!(transcribe(wav, 0), Err(MelodyError::NoMelody));
    }

    #[test]
    fn rejects_invalid_octave() {
        assert_eq!(
            transcribe(Vec::new(), MAX_OCTAVE + 1),
            Err(MelodyError::InvalidOctave(MAX_OCTAVE + 1))
        );
    }

    #[test]
    fn rejects_non_audio() {
        let r = transcribe(b"definitely not audio".to_vec(), 0);
        assert!(matches!(r, Err(MelodyError::Decode(_))), "{r:?}");
    }

    #[test]
    fn decodes_wav_end_to_end() {
        let wav = wav_bytes(&synth(&[(440.0, 400), (523.25, 400)]));
        let t = transcribe(wav, 0).unwrap();
        let freqs: Vec<u16> = t.notes.iter().map(|n| n.freq).collect();
        assert_eq!(freqs, vec![A4, C5]);
        assert!(!t.truncated);
    }

    #[test]
    fn play_needs_a_song() {
        let mut s = MelodyState::default();
        assert_eq!(s.play(), Err(MelodyError::NothingLoaded));
        assert_eq!(s.version, 0);
    }

    /// Minimal 16-bit mono PCM WAV file.
    fn wav_bytes(samples: &[f32]) -> Vec<u8> {
        let data_len = (samples.len() * 2) as u32;
        let mut v = Vec::new();
        v.extend_from_slice(b"RIFF");
        v.extend_from_slice(&(36 + data_len).to_le_bytes());
        v.extend_from_slice(b"WAVEfmt ");
        v.extend_from_slice(&16u32.to_le_bytes());
        v.extend_from_slice(&1u16.to_le_bytes()); // PCM
        v.extend_from_slice(&1u16.to_le_bytes()); // mono
        v.extend_from_slice(&RATE.to_le_bytes());
        v.extend_from_slice(&(RATE * 2).to_le_bytes());
        v.extend_from_slice(&2u16.to_le_bytes());
        v.extend_from_slice(&16u16.to_le_bytes());
        v.extend_from_slice(b"data");
        v.extend_from_slice(&data_len.to_le_bytes());
        for s in samples {
            v.extend_from_slice(&((s * i16::MAX as f32) as i16).to_le_bytes());
        }
        v
    }
}
