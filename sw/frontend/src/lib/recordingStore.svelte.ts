/** User-controlled capture toggle. Samples remain in the bounded chart buffers. */
class RecordingStore {
  isRecording = $state(false);
}

export const recordingStore = new RecordingStore();
