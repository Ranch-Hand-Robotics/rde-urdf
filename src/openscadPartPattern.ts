const OPENSCAD_PART_PATTERN_TOKEN = 'MxN';
const MAX_PATTERN_PART_PROBES = 10_000;

export function isOpenSCADPartPattern(partName: string): boolean {
    return partName.includes(OPENSCAD_PART_PATTERN_TOKEN);
}

export function formatOpenSCADPartPattern(partPattern: string, m: number, n: number): string {
    return partPattern.replace(OPENSCAD_PART_PATTERN_TOKEN, `${m}x${n}`);
}

/**
 * Probe an MxN part pattern in row-major order, with N as the inner dimension.
 * An empty probe ends the current M row. An empty probe at N=0 ends the pattern.
 */
export async function iterateOpenSCADPartPattern(
    partPattern: string,
    probe: (partName: string, m: number, n: number) => Promise<boolean>,
    maxProbes: number = MAX_PATTERN_PART_PROBES,
): Promise<string[]> {
    if (!isOpenSCADPartPattern(partPattern)) {
        throw new Error(`Part '${partPattern}' does not contain the ${OPENSCAD_PART_PATTERN_TOKEN} pattern.`);
    }
    if (!Number.isInteger(maxProbes) || maxProbes <= 0) {
        throw new Error('The maximum MxN part probe count must be a positive integer.');
    }

    const generatedParts: string[] = [];
    let probeCount = 0;

    for (let m = 0; ; m++) {
        for (let n = 0; ; n++) {
            if (probeCount >= maxProbes) {
                throw new Error(
                    `Part pattern '${partPattern}' exceeded the ${maxProbes} probe safety limit. ` +
                    'Ensure the OpenSCAD model eventually produces no output for each dimension.'
                );
            }

            const partName = formatOpenSCADPartPattern(partPattern, m, n);
            probeCount++;
            const generated = await probe(partName, m, n);
            if (!generated) {
                if (n === 0) {
                    return generatedParts;
                }
                break;
            }

            generatedParts.push(partName);
        }
    }
}
