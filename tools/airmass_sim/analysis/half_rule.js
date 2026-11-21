// AirMass's static_comp from the TE compensation setting by the half rule, 100 + (adjust − 100) / 2, against its
// optimum, per installation, from te_adjust.js outputs.
//   node half_rule.js TE_ADJUST.json...      (EXCLUDE=file,...: logs left out)
// An installation is a glider type and, for the Arcus, the month (its flights wanted different settings). Its setting
// is eComp's best (the pilot's choice, the speed-change windows of all its flights); AirMass is scored with the half
// rule's value, its own best, the setting itself (what the firmware does now) and 100 (the replays' tuning). Values
// between the swept ones are interpolated in the mean square.
const exclude = new Set((process.env.EXCLUDE || "").split(",").filter(Boolean));
const flights = process.argv.slice(2).flatMap((f) => require(require("path").resolve(f)).flights).filter((r) => r.sums && r.sums.all && !exclude.has(r.file));
const installation = (r) => (r.glider === "Arcus" ? `Arcus ${r.file.slice(0, 7)}` : r.glider);
const PHASES = ["speed change", "all"];

// pooled mean squares per series and phase
function pool(rows) {
  const P = {};
  for (const r of rows) for (const ph of PHASES) {
    const a = r.sums[ph];
    if (!a) continue;
    const p = (P[ph] ||= { n: 0 });
    for (const k in a) p[k] = (p[k] || 0) + a[k];
  }
  return P;
}
const values = (P, prefix) => Object.keys(P.all).filter((k) => k.startsWith(prefix + " ")).map((k) => +k.split(" ")[1]).sort((a, b) => a - b);
// rms of a method at a setting, interpolated in the mean square between the swept values
function at(P, ph, prefix, v) {
  const vs = values(P, prefix), ms = (x) => P[ph][`${prefix} ${x}`] / P[ph].n;
  if (v <= vs[0]) return Math.sqrt(ms(vs[0]));
  if (v >= vs[vs.length - 1]) return Math.sqrt(ms(vs[vs.length - 1]));
  const i = vs.findIndex((x) => x >= v), a = vs[i - 1], b = vs[i];
  return v === b ? Math.sqrt(ms(b)) : Math.sqrt(ms(a) + (ms(b) - ms(a)) * (v - a) / (b - a));
}
const best = (P, ph, prefix) => values(P, prefix).reduce((b, v) => (at(P, ph, prefix, v) < at(P, ph, prefix, b) ? v : b));
const half = (e) => 100 + (e - 100) / 2;

const groups = {};
for (const r of flights) (groups[installation(r)] ||= []).push(r);
const f3 = (v) => v.toFixed(3), pct = (v, o) => `${v >= o ? "+" : ""}${(100 * (v / o - 1)).toFixed(1)} %`;
for (const ph of PHASES) {
  console.log(`\n${ph} windows, 10 s rms m/s (AirMass against its own best)\n`);
  console.log("| installation (flights, windows) | eComp best | half rule | AirMass at it | AirMass best | at the setting | at 100 |\n|---|---|---|---|---|---|---|");
  const tot = { n: 0, half: 0, best: 0, same: 0, hundred: 0 };
  for (const [name, rows] of Object.entries(groups)) {
    const P = pool(rows);
    if (!P[ph]) continue;
    const e = best(P, "speed change", "eComp"), h = half(e), ab = best(P, ph, "AirMass");
    const r = { half: at(P, ph, "AirMass", h), best: at(P, ph, "AirMass", ab), same: at(P, ph, "AirMass", e), hundred: at(P, ph, "AirMass", 100) };
    for (const k in r) tot[k] += r[k] ** 2 * P[ph].n;
    tot.n += P[ph].n;
    console.log(`| ${name} (${rows.length}, ${P[ph].n}) | ${e} (${f3(at(P, ph, "eComp", e))}) | ${h} | ${f3(r.half)} (${pct(r.half, r.best)}) | ${ab} (${f3(r.best)}) | ${f3(r.same)} (${pct(r.same, r.best)}) | ${f3(r.hundred)} (${pct(r.hundred, r.best)}) |`);
  }
  const t = (k) => Math.sqrt(tot[k] / tot.n);
  console.log(`| all, each at its own setting | | | ${f3(t("half"))} (${pct(t("half"), t("best"))}) | ${f3(t("best"))} | ${f3(t("same"))} (${pct(t("same"), t("best"))}) | ${f3(t("hundred"))} (${pct(t("hundred"), t("best"))}) |`);
  // everyone at the default 92
  const P = pool(flights);
  console.log(`| all at the default 92 (eComp ${f3(at(P, ph, "eComp", 92))}) | | 96 | ${f3(at(P, ph, "AirMass", 96))} | ${best(P, ph, "AirMass")} (${f3(at(P, ph, "AirMass", best(P, ph, "AirMass")))}) | ${f3(at(P, ph, "AirMass", 92))} | ${f3(at(P, ph, "AirMass", 100))} |`);
}
