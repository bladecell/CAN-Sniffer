import { describe, expect, test } from "bun:test";
import { PLOT_CSV_HEADER, buildPlotCsv, csvField, formatCsvValue, plotCsvFileName } from "./plotCsv";

const rpm = { pid: 12, name: "Engine RPM", unit: "rpm", t: [100, 300], v: [800, 1250.5] as (number | null)[] };
const temp = { pid: 5, name: "Coolant, temp", unit: "C", t: [200], v: [-3] as (number | null)[] };

describe("buildPlotCsv", () => {
  test("header only when the plot has no series", () => {
    expect(buildPlotCsv([], null)).toBe("t_ms,pid,value,valid,name,unit\n");
    expect(PLOT_CSV_HEADER).toBe("t_ms,pid,value,valid,name,unit");
  });

  test("merges several PIDs into one long table ordered by time, escaping the name", () => {
    expect(buildPlotCsv([rpm, temp], null)).toBe(
      [
        "t_ms,pid,value,valid,name,unit",
        "100,12,800,1,Engine RPM,rpm",
        '200,5,-3,1,"Coolant, temp",C',
        "300,12,1250.5,1,Engine RPM,rpm",
        "",
      ].join("\n"),
    );
  });

  test("an empty series contributes no rows", () => {
    const empty = { pid: 7, name: "Empty", unit: "", t: [], v: [] };
    expect(buildPlotCsv([empty], null)).toBe("t_ms,pid,value,valid,name,unit\n");
  });

  test("null, NaN and infinite values are written empty with valid=0", () => {
    const s = { pid: 5, name: "T", unit: "C", t: [10, 20, 30, 40], v: [null, NaN, Infinity, -Infinity] };
    expect(buildPlotCsv([s], null)).toBe(
      ["t_ms,pid,value,valid,name,unit", "10,5,,0,T,C", "20,5,,0,T,C", "30,5,,0,T,C", "40,5,,0,T,C", ""].join("\n"),
    );
  });

  test("a zoomed domain keeps only samples inside it, both ends inclusive", () => {
    expect(buildPlotCsv([rpm, temp], { fromT: 200, toT: 300 })).toBe(
      ["t_ms,pid,value,valid,name,unit", '200,5,-3,1,"Coolant, temp",C', "300,12,1250.5,1,Engine RPM,rpm", ""].join("\n"),
    );
  });

  test("samples with the same time keep the plot's series order", () => {
    const a = { pid: 1, name: "A", unit: "", t: [50], v: [1] as (number | null)[] };
    const b = { pid: 2, name: "B", unit: "", t: [50], v: [2] as (number | null)[] };
    expect(buildPlotCsv([b, a], null).split("\n").slice(1, 3)).toEqual(["50,2,2,1,B,", "50,1,1,1,A,"]);
  });

  test("a non-finite time is dropped and a mismatched series uses the shorter length", () => {
    const s = { pid: 3, name: "S", unit: "", t: [Number.NaN, 60, 70], v: [1, 2] as (number | null)[] };
    expect(buildPlotCsv([s], null)).toBe(["t_ms,pid,value,valid,name,unit", "60,3,2,1,S,", ""].join("\n"));
  });
});

describe("formatCsvValue (%.8g as the recorder writes it)", () => {
  test.each([
    [1, "1"],
    [0.5, "0.5"],
    [-0, "0"],
    [1 / 3, "0.33333333"],
    [12345678, "12345678"],
    [123456780, "1.2345678e+08"],
    [0.0001234, "0.0001234"],
    [0.00001234, "1.234e-05"],
    [99999999.5, "1e+08"],
    [1e21, "1e+21"],
    [-1.5e-7, "-1.5e-07"],
  ])("%p -> %p", (input, expected) => {
    expect(formatCsvValue(input)).toBe(expected);
  });
});

describe("csvField", () => {
  test("plain text passes through", () => {
    expect(csvField("Engine RPM")).toBe("Engine RPM");
  });

  test("commas, quotes and line breaks are quoted with doubled quotes", () => {
    expect(csvField("a,b")).toBe('"a,b"');
    expect(csvField('say "hi"')).toBe('"say ""hi"""');
    expect(csvField("line\nbreak")).toBe('"line\nbreak"');
  });

  test("a leading = + - or @ gets an apostrophe so spreadsheets do not run it as a formula", () => {
    expect(csvField("=1+1")).toBe("'=1+1");
    expect(csvField("+x")).toBe("'+x");
    expect(csvField("-x")).toBe("'-x");
    expect(csvField("@SUM(A1)")).toBe("'@SUM(A1)");
    expect(csvField(" =1")).toBe("' =1");
  });

  test("the apostrophe guard and the quoting combine", () => {
    expect(csvField("=a,b")).toBe("\"'=a,b\"");
  });
});

describe("plotCsvFileName", () => {
  test("plot number and local time to the second", () => {
    expect(plotCsvFileName(2, new Date(2026, 10, 2, 9, 5, 7))).toBe("can-sniffer-plot-2-20261102-090507.csv");
  });
});
