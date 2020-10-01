import { useEffect, useState } from "react"
import { useAppContext } from "../providers"
import { initROS } from "../utils/rosConfig"

export const useRos = () => {
    const { state, updateConnectionStatus } = useAppContext();
    const { isConnected } = state;
    const [ros, setRos] = useState<ROSLIB.Ros>()
    const connect = () => {
        if (!isConnected) {
            setRos(initROS(updateConnectionStatus))
            // ros?.connect(rosconfig.url)
        } else {
            console.log("Already connected")
        }
    }
    useEffect(() => {
        connect()
        return () => (ros)?.close()
        // eslint-disable-next-line react-hooks/exhaustive-deps 
    }, [])
    return { ros, connect }
}