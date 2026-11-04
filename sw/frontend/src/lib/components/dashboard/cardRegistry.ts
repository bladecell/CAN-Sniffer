// One body component per card type. Each body reads its own style from `card.style`.
import type { Component } from "svelte";
import type { Card, CardType } from "$lib/dashboardModel";
import BarCard from "./cards/BarCard.svelte";
import BatteryCard from "./cards/BatteryCard.svelte";
import ChartCard from "./cards/ChartCard.svelte";
import DtcCard from "./cards/DtcCard.svelte";
import GaugeCard from "./cards/GaugeCard.svelte";
import OverviewCard from "./cards/OverviewCard.svelte";
import PollingCard from "./cards/PollingCard.svelte";
import RecorderCard from "./cards/RecorderCard.svelte";
import StatusCard from "./cards/StatusCard.svelte";
import ValueCard from "./cards/ValueCard.svelte";

export const cardBodies: Record<CardType, Component<{ card: Card }>> = {
  value: ValueCard,
  bar: BarCard,
  gauge: GaugeCard,
  chart: ChartCard,
  battery: BatteryCard,
  overview: OverviewCard,
  dtc: DtcCard,
  status: StatusCard,
  polling: PollingCard,
  recorder: RecorderCard,
};
