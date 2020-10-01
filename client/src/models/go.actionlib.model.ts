export interface Pose2D {
    x: number
    y: number
    theta: number
}
const r = (n: number) => Math.round(n)
export const Pose2DisEqual = (a: Pose2D, b: Pose2D): boolean => {
    const res = r(a.x) === r(b.x) && r(a.y) === r(b.y)
    console.log(`${(a.x)} === ${(b.x)} && ${(a.y)} === ${(b.y)}`)
    return res;
}

export type GoActionFeedback = { percentage: number }
export type GoActionResult = { time_elapsed: { secs: number, nsecs: number } }
export type GoActionGoal = Pose2D