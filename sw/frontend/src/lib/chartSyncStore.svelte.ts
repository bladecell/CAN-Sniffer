export class ChartSyncStore {
  syncEnabled = $state(true);
  domain: [number, number] | null = $state(null);
}

export const chartSyncStore = new ChartSyncStore();
