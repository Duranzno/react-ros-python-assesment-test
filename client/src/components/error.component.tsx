import React, { useState } from 'react'
import { Button, Container, Grid, Typography } from '@material-ui/core'
import RefreshIcon from '@material-ui/icons/Refresh';
import { makeStyles } from '@material-ui/styles';
const useStyles = makeStyles({
    root: {
        height: "100%",
        minWidth: "30vw",
        minHeight: "50vh",
    },
    dino: {
        height: 64,
        width: 64,
        alignSelf: "center",
        background: "url('https://s3-us-west-2.amazonaws.com/s.cdpn.io/108721/DecodedBase64.png')",
    },
    paragraph: {
        whiteSpace: "pre-line"
    },
})
export const NotConnected: React.FC<{ onRefresh: () => void }> = ({ onRefresh }) => {
    const classes = useStyles()
    const [moreInfo, setMoreInfo] = useState(false)
    const toggle = () => setMoreInfo(!moreInfo)
    return (
        <Container maxWidth="md" className={classes.root}>
            <Grid container direction="column" justify="space-between" alignContent="center" spacing={3}>

                <div className={classes.dino}></div>
                <Typography >Seems you are not connected to a ROS Bridge Server</Typography >
                <Button variant="text" onClick={toggle}> More Info </Button>
                <Grid item>
                    {moreInfo && (<>
                        <Typography className={classes.paragraph}>{`You should check if ROS Bridge is running
                    The best way to run the entire project is to use 
                    the run_with_bridge.launch file 
                    `}
                        </Typography>
                    </>
                    )}
                </Grid>
                <Button endIcon={<RefreshIcon />} size="large" variant="contained" aria-label="refresh" color="primary" onClick={onRefresh}>
                    Try to connect again
            </Button>
            </Grid>
        </Container>
    )
}
