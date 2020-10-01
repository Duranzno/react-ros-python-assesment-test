import { useCallback, useEffect } from "react"
import { DynaRecParams, DynaRecRes, DynaRecTuple } from "../models"
import ROSLIB from 'roslib'
import { useAppContext } from "../providers"

const getDynaRec = (config: DynaRecRes, type: DynaRecParams = DynaRecParams.SPEED): DynaRecTuple => {
    return config.doubles.find(v => v.name === type) ?? { name: "", value: 0 }
}

// eslint-disable-next-line @typescript-eslint/explicit-function-return-type
export const useSpeedModifier = (ros: ROSLIB.Ros) => {
    const { updateSpeed: setSpeed, getSpeedConfig: setConfig, state: { speedConfig } } = useAppContext()
    const dynaRecClient = new ROSLIB.Service({
        ros: ros,
        name: '/go_to_goal/set_parameters',
        serviceType: 'dynamic_reconfigure/Reconfigure'
    });
    const confiData = new ROSLIB.Topic({
        ros,
        name: "/go_to_goal/parameter_descriptions",
        messageType: 'dynamic_reconfigure/ConfigDescription'
    })

    const onSubscribe = useCallback((_data: ROSLIB.Message): void => {
        // console.log(_data)
        if (speedConfig.max === 0) {
            const data = _data as {
                dflt: DynaRecRes,
                groups: DynaRecRes,
                max: DynaRecRes,
                min: DynaRecRes
            }
            const min = getDynaRec(data.min).value
            const max = getDynaRec(data.max).value
            const dflt = getDynaRec(data.dflt).value
            setConfig({ max, min, dflt })
            setSpeed(dflt)
        }
        // console.log(min, max, dflt)
    }, [speedConfig.max, setConfig, setSpeed])
    useEffect(() => {
        if (speedConfig.max !== 0) {
            confiData.unsubscribe()
        } else {
            confiData.subscribe(onSubscribe)
        }
    }, [speedConfig, confiData, onSubscribe])
    const changeSpeed = (speed: number): void => {
        setSpeed(speed)
        const request = new ROSLIB.ServiceRequest({
            config: {
                doubles: [
                    {
                        name: "speed_param",
                        value: speed
                    }
                ]
            }
        });
        const cb = (result: { config: DynaRecRes }): void => {
            // console.log("Sucessfull dynarec change", result.config)
            const speed = getDynaRec(result.config)
            if (speed?.value) setSpeed(speed.value)
        }
        dynaRecClient.callService(request, cb, console.error);
    }
    return { changeSpeed }
}

