use nnnoiseless::DenoiseState;

pub(crate) trait Filter {
    fn filter_audio(&mut self, input: &Vec<i16>, output: &mut Vec<i16>);
}

impl Filter for DenoiseState<'_> {
    fn filter_audio(&mut self, input: &Vec<i16>, output: &mut Vec<i16>) {
        let mut out_samples_float = [0f32; DenoiseState::FRAME_SIZE];
        assert_eq!(input.len(), DenoiseState::FRAME_SIZE);
        let input_samples_float: Vec<f32> = input.iter()
            .map(|value| *value as f32)
            .collect();
        self.process_frame(&mut out_samples_float, &input_samples_float,
        );
        out_samples_float.iter()
            .map(|value| *value as i16)
            .for_each(|value| output.push(value));
    }
}
