import ROSLIB from 'roslib'
import { useAppContext } from '../providers';

interface PauseServiceResponse {
    is_paused: boolean
}

// eslint-disable-next-line @typescript-eslint/explicit-function-return-type
export const useToggle = (ros: ROSLIB.Ros) => {
    const { togglePause, state: { isPaused } } = useAppContext()

    const pauseService = new ROSLIB.Service({
        ros: ros,
        name: '/turtle1/pause',
        serviceType: 'backend/Pause'
    });

    const toggle = (): void => {
        const reqPause = new ROSLIB.ServiceRequest({
            "is_paused": isPaused,
        });
        const cb = (result: PauseServiceResponse): void => {
            console.log(`Server was told to ${isPaused ? 'pause' : 'resume'}`)
            console.log(`Server answered that it is now ${result.is_paused ? 'paused' : 'resumed'}`)
            togglePause(result.is_paused)
        }
        pauseService.callService(reqPause, cb);
    }
    return { toggle }
}
