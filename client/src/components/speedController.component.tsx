import React from 'react';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
import Slider from '@material-ui/core/Slider';
import Input from '@material-ui/core/Input';
import SpeedIcon from '@material-ui/icons/Speed';


interface Props {
    changeSpeed: (speed: number) => void
    max: number,
    min: number,
    currentSpeed: number,
}
export const SpeedController: React.FC<Props> = ({ changeSpeed, max, min, currentSpeed: value }) => {

    const setValue = (val: number | number[]) => {
        const newVal = (Array.isArray(val) ? val.pop() : val) ?? min;
        changeSpeed(newVal)
    }
    const handleSliderChange = (_event: React.ChangeEvent<{}>, newValue: number | number[]) => {
        setValue(newValue);
    };

    const handleInputChange = (event: React.ChangeEvent<HTMLTextAreaElement | HTMLInputElement>) => {
        setValue(event.target.value === '' ? value : Number(event.target.value));
    };

    const handleBlur = () => {
        if (value < 0) {
            setValue(0);
        } else if (value > 100) {
            setValue(100);
        }
    };
    return (
        <Grid container spacing={2} alignItems="center" direction="column">
            <Typography id="input-slider" gutterBottom>
                Speed
            </Typography>
            <Grid container spacing={2} alignItems="center">
                <Grid item>
                    <SpeedIcon />
                </Grid>
                <Grid item xs>
                    <Slider
                        value={typeof value === 'number' ? value : 0}
                        onChange={handleSliderChange}
                        aria-labelledby="input-slider"
                        step={10}
                        max={max}
                        min={min}
                    />
                </Grid>
                <Grid item>
                    <Input
                        value={value}
                        margin="dense"
                        onChange={handleInputChange}
                        onBlur={handleBlur}
                        inputProps={{
                            step: 10,
                            min,
                            max,
                            type: 'number',
                            'aria-labelledby': 'input-slider',
                        }}
                    />
                </Grid>
            </Grid>
        </Grid>
    )
}