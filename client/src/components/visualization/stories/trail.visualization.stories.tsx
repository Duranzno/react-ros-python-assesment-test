// Button.stories.tsx

import React from 'react';
import { Meta } from '@storybook/react/types-6-0';
import { MAX_SIDE_WEB, PIXI_CENTER_WEB } from '../../../constants';
import { Stage, Sprite } from '@inlet/react-pixi';
import { Point, Texture } from 'pixi.js';
import { TrailVisualization } from '../trail.visualization.component';

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

const TrailTemplate: React.FC<{ x: number, y: number }> = ({ x, y }) => {
    return <TrailVisualization point={new Point(x, y)} />
}
export const Story = TrailTemplate.bind({});



const control = { type: "number", min: 0, max: MAX_SIDE_WEB, step: 10 }
export default {
    title: '2DVisualization/Trail',
    component: TrailVisualization,
    args: {
        x: PIXI_CENTER_WEB.x,
        y: PIXI_CENTER_WEB.y,
    },
    argTypes: {
        x: {
            defaultValue: PIXI_CENTER_WEB.x,
            control,
        },
        y: {
            defaultValue: PIXI_CENTER_WEB.y,
            control,
        }
    },
    decorators: [(Story) => (<Decorator>
        <Story />
    </Decorator>)]
} as Meta;
