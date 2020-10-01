import React, { useReducer, Dispatch as D } from 'react';
import { SpeedConfig } from '../models';
import createCtx from '../utils/createCtx';

export interface State {
    isConnected: boolean
    currentSpeed: number
    speedConfig: SpeedConfig,
    isPaused: boolean,
    showTrailPoints: boolean,
}

const initialState: State = {
    isConnected: false,
    currentSpeed: 0,
    speedConfig: { dflt: 0, min: 0, max: 0 },
    isPaused: false,
    showTrailPoints: true,
};
interface Context {
    state: State;
    updateConnectionStatus: (isConnected: boolean) => void;
    updateSpeed: (speed: number) => void;
    togglePause: (toggleState: boolean) => void;
    toggleTrailPoints: (showTrailPoints: boolean) => void;
    getSpeedConfig: (config: SpeedConfig) => void
}
const [useCtx, Provider] = createCtx<Context>();

export enum ActionType {
    CONNECT = "updateConnectionStatus",
    SPEED = "updateSpeed",
    PAUSE = "togglePause",
    SPEED_CONFIG = "getSpeedConfig",
    TRAIL_POINTS = "showTrailPoints"
}

type ConnectAction = { type: ActionType.CONNECT, payload: boolean }
type SpeedAction = { type: ActionType.SPEED, payload: number }
type PauseAction = { type: ActionType.PAUSE, payload: boolean }
type TrailPointsAction = { type: ActionType.TRAIL_POINTS, payload: boolean }
type SpeedConfigAction = { type: ActionType.SPEED_CONFIG, payload: SpeedConfig }

export type Action =
    | ConnectAction
    | SpeedAction
    | PauseAction
    | SpeedConfigAction
    | TrailPointsAction


type Reducer = (state: State, action: Action) => State;

const connect = (dispatch: D<ConnectAction>) => (payload: boolean): void => {
    dispatch({ type: ActionType.CONNECT, payload, });
};
const changeSpeed = (dispatch: D<SpeedAction>) => (payload: number): void => {
    dispatch({ type: ActionType.SPEED, payload, });
};
const toggle = (dispatch: D<PauseAction>) => (payload: boolean): void => {
    dispatch({ type: ActionType.PAUSE, payload, });
};
const toggleTrailPoints = (dispatch: D<TrailPointsAction>) => (payload: boolean): void => {
    dispatch({ type: ActionType.TRAIL_POINTS, payload, });
};
const getSpeedConfig = (dispatch: D<SpeedConfigAction>) => (payload: SpeedConfig): void => {
    dispatch({ type: ActionType.SPEED_CONFIG, payload, });
};


const reducer: Reducer = (state = initialState, action: Action) => {
    switch (action.type) {
        case ActionType.CONNECT:
            return { ...state, isConnected: action.payload }
        case ActionType.PAUSE:
            return { ...state, isPaused: action.payload }
        case ActionType.TRAIL_POINTS:
            return { ...state, showTrailPoints: action.payload }
        case ActionType.SPEED:
            return { ...state, currentSpeed: action.payload }
        case ActionType.SPEED_CONFIG:
            return { ...state, speedConfig: action.payload }
        default:
            return state;
    }
};

const AppProvider: React.FC = (props) => {
    const [state, dispatch] = useReducer<Reducer>(
        reducer,
        initialState,
    );

    const actions = {
        updateConnectionStatus: connect(dispatch),
        updateSpeed: changeSpeed(dispatch),
        togglePause: toggle(dispatch),
        toggleTrailPoints: toggleTrailPoints(dispatch),
        getSpeedConfig: getSpeedConfig(dispatch)
    };

    return (
        <Provider value={{ state, ...actions }}>
            {props.children}
        </Provider>
    );
};

export { useCtx as useAppContext, AppProvider };
