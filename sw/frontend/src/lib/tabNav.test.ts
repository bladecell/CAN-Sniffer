import { expect, test } from "bun:test";
import { tabTarget } from "./tabNav";

const ids = ["a", "b", "c"];

test("arrows move and wrap", () => {
  expect(tabTarget(ids, "a", "ArrowRight")).toBe("b");
  expect(tabTarget(ids, "c", "ArrowRight")).toBe("a");
  expect(tabTarget(ids, "a", "ArrowLeft")).toBe("c");
});

test("Home and End jump", () => {
  expect(tabTarget(ids, "b", "Home")).toBe("a");
  expect(tabTarget(ids, "b", "End")).toBe("c");
});

test("other keys are ignored", () => {
  expect(tabTarget(ids, "a", "Enter")).toBeUndefined();
  expect(tabTarget([], "a", "Home")).toBeUndefined();
});
