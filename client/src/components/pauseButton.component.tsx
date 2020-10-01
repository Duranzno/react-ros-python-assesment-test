import { Grid, Switch, Typography } from '@material-ui/core'
import React from 'react'
interface Props {
    toggle: () => void,
    paused: boolean,

}

export const PauseButton: React.FC<Props> = ({ toggle, paused }) => {
    return (
        <Grid container spacing={2} alignItems="center" direction="column">
            <Typography id="input-slider" gutterBottom>
                {!paused ? "Pause" : "Resume"}
            </Typography>
            <Switch checked={!paused} onChange={() => toggle()} />
        </Grid>
    )
}