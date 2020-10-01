// Button.stories.tsx

import React from 'react';
import { Meta } from '@storybook/react/types-6-0';
import { Stage, Sprite } from '@inlet/react-pixi';
import { MAX_SIDE_WEB, PIXI_CENTER_WEB } from '../../../constants';
import { Texture } from 'pixi.js';
import { MapVisualization } from '../map.visualization.component'

const MapTemplate: React.FC<{ x: number, y: number }> = ({ x, y }) => {
    return <MapVisualization
        onNewGoal={(v) => console.log(v)}
        turtleposition={{
            x, y, theta: 0
        }} />
}

const control = { type: "number", min: 0, max: MAX_SIDE_WEB, step: 10 }

export const Story = MapTemplate.bind({});

export default {
    title: '2DVisualization/MapVisualization',
    component: MapVisualization,
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
    // decorators: [(Story) => (<Decorator>
    // <Story />
    // </Decorator>)]
} as Meta;
