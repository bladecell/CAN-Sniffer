import { describe, expect, test } from "bun:test";
import { blurBlockers, type AncestorStyle } from "./layers";

const plain: AncestorStyle = {
  transform: "none",
  opacity: "1",
  filter: "none",
  clipPath: "none",
  maskImage: "none",
  backdropFilter: "none",
  overflowX: "visible",
  overflowY: "visible",
};

describe("blur blockers", () => {
  test("a plain box cannot drop a float layer's blur", () => {
    expect(blurBlockers(plain)).toEqual([]);
  });
  test("a scroller, a clipped box or a blurred box each drop it", () => {
    expect(blurBlockers({ ...plain, overflowY: "auto" })).toEqual(["overflow"]);
    expect(blurBlockers({ ...plain, overflowX: "hidden" })).toEqual(["overflow"]);
    expect(blurBlockers({ ...plain, backdropFilter: "blur(26px)" })).toEqual(["backdrop-filter"]);
  });
  test("transforms, translucency, filters and masks each drop it", () => {
    expect(blurBlockers({ ...plain, transform: "matrix(1, 0, 0, 1, 0, 8)" })).toEqual(["transform"]);
    expect(blurBlockers({ ...plain, opacity: "0.5" })).toEqual(["opacity"]);
    expect(blurBlockers({ ...plain, filter: "blur(1px)" })).toEqual(["filter"]);
    expect(blurBlockers({ ...plain, clipPath: "inset(0px)" })).toEqual(["clip-path"]);
    expect(blurBlockers({ ...plain, maskImage: "linear-gradient(#000, #000)" })).toEqual(["mask"]);
  });
});
