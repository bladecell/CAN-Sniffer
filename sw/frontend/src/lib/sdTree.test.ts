import { expect, test } from "bun:test";
import { nodeKey } from "./sdTree";

test("sibling files without a path get distinct keys", () => {
  const files = [{ name: "a.csv", type: "file" }, { name: "b.csv", type: "file" }];
  expect(new Set(files.map((f) => nodeKey(f as any))).size).toBe(2);
});

test("a directory keys by path", () => {
  expect(nodeKey({ path: "/trips", children: [] } as any)).toBe("/trips");
});
