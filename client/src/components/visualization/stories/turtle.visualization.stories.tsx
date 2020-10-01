// Button.stories.tsx

import React from 'react';
import { Meta } from '@storybook/react/types-6-0';
import { Stage, Sprite } from '@inlet/react-pixi';
import { Point, Texture } from 'pixi.js';
import { MAX_SIDE_WEB, PIXI_CENTER_WEB } from '../../../constants';
import { TurtleVisualization } from './../turtle.visualization.component';
import { fixPixiTheta } from '../../../models';

const Decorator: React.FC = ({ children }) => {
    return (
        <Stage
            style={{ margin: '3em' }}
            width={MAX_SIDE_WEB}
            height={MAX_SIDE_WEB}
            options={{ transparent: true }}>
            <Sprite width={MAX_SIDE_WEB} height={MAX_SIDE_WEB}
                texture={Texture.WHITE}
                tint={0x4556FF} />
            {children}
        </Stage>)
}

const TurtleTemplate: React.FC<{ x: number, y: number, theta: number }> = ({ x, y, theta }) => {
    return <TurtleVisualization point={{ x, y, theta: fixPixiTheta(theta) }} />
}
export const Story = TurtleTemplate.bind({});


const pi = Math.PI
const control = { type: "number", min: 0, max: MAX_SIDE_WEB, step: 10 }
export default {
    title: '2DVisualization/Turtle',
    component: TurtleVisualization,
    args: {
        x: PIXI_CENTER_WEB.x,
        y: PIXI_CENTER_WEB.y,
        theta: 0,
    },
    argTypes: {
        x: {
            defaultValue: PIXI_CENTER_WEB.x,
            control,
        },
        y: {
            defaultValue: PIXI_CENTER_WEB.y,
            control,
        },
        theta: {
            defaultValue: 0,
            control: {
                type: "number",
                min: 0,
                max: 2 * pi,
                step: pi / 6,
                options: [
                    [
                        pi / 6,
                        pi / 3,
                        pi / 2,
                        4 * pi / 6,
                        5 * pi / 6,
                        pi,
                        7 * pi / 6,
                        8 * pi / 6,
                        9 * pi / 6,
                        10 * pi / 6,
                        11 * pi / 6,
                        2 * pi,
                    ]
                ]
            }
        }
    },
    decorators: [(Story) => (<Decorator>
        <Story />
    </Decorator>)]
} as Meta;
