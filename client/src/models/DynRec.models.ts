export type DynaRecTuple = { name: string, value: number }
export type DynaRecRes = {
    doubles: Array<DynaRecTuple>
}
export enum DynaRecParams { SPEED = "speed_param" }
export interface SpeedConfig {
    min: number;
    max: number;
    dflt: number;
}
