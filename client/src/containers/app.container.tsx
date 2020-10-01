import React from 'react'
import { Grid, Paper, Container } from '@material-ui/core'
import { makeStyles } from '@material-ui/styles';



const useStyles = makeStyles({
    root: {
        flexGrow: 1,
        overflow: "hidden",
        minWidth: "100vw",
        minHeight: "100vh",
    },
    container: {
        minWidth: "640px",
        maxHeight: "640px",
        minHeight: "640px"
    },
    paper: {
        height: "100%",
        width: "100%",
    }
});
export const AppContainer: React.FC = ({ children }) => {
    const classes = useStyles();

    return (
        <Grid container className={classes.root} justify="center" alignContent="center">

            <Container maxWidth="md" className={classes.container}>

                <Paper className={classes.paper} >
                    <Grid className={classes.paper} container justify="center" alignContent="center" >
                        <Grid item >
                            {children}
                        </Grid>
                    </Grid>
                </Paper>
            </Container>

        </Grid>

    )
}
