import React from 'react'
import ROSLIB from 'roslib'

import {
    Grid,
    // Switch, 
    // Typography 
} from '@material-ui/core';
import { MapController, PauseButton, SpeedController } from '../components';
import { useSpeedModifier, useToggle } from '../hooks';
import { useAppContext } from '../providers';
import { makeStyles } from '@material-ui/styles';


const useStyles = makeStyles({
    root: {
        padding: "1rem"
    },
    map: {
        padding: "0.5rem",
        paddingBottom: "1rem"
    }
});
interface Props {
    ros: ROSLIB.Ros
}
export const RosContainer: React.FC<Props> = ({ ros }) => {
    const {
        state: {
            isConnected,
            isPaused,
            currentSpeed,
            // showTrailPoints,
            speedConfig: {
                max,
                min
            },
        },
        // toggleTrailPoints
    } = useAppContext();
    const { toggle } = useToggle(ros)
    const { changeSpeed } = useSpeedModifier(ros)
    const classes = useStyles()
    return (
        <Grid container className={classes.root} justify="center" spacing={2} direction="column">
            <MapController ros={ros} className={classes.map} isConnected={isConnected} />
            <Grid container spacing={2} direction="row" alignItems="center" justify="space-between">
                <Grid item xs={6} sm={3} >
                    <PauseButton toggle={toggle} paused={isPaused} />
                </Grid>
                {/* <Grid item xs={6} sm={2} container spacing={2} alignItems="center" direction="column">
                    <Typography id="input-slider" gutterBottom>
                        {showTrailPoints ? "Hide Dots" : "Show Dots"}
                    </Typography>
                    <Switch checked={showTrailPoints} onChange={() => toggleTrailPoints(!showTrailPoints)} />
                </Grid> */}
                <Grid item xs={12} sm={8} >
                    <SpeedController
                        changeSpeed={changeSpeed}
                        max={max}
                        min={min}
                        currentSpeed={currentSpeed}
                    />
                </Grid>
            </Grid>
        </Grid>
    )
}
