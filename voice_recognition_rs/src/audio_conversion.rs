use crossbeam_channel::{Sender, SendError};

pub(crate) struct AudioConverter {
    pub(crate) input_sample_rate: u32,
    pub(crate) output_sample_rate: u32,
    pub(crate) sender: Sender<Vec<i16>>,
}

impl AudioConverter {
    pub(crate) fn send(&self, data: &[i16]) -> Result<(), SendError<Vec<i16>>> {
        if self.input_sample_rate == self.output_sample_rate {
            self.sender.send(data.to_owned())
        } else {
            let factor = self.input_sample_rate / self.output_sample_rate;

            let iter = data.chunks_exact(factor as usize);
            let resampled_data = iter
                .map(|chunk| chunk[0])
                .collect();

            self.sender.send(resampled_data)
        }
    }
}
