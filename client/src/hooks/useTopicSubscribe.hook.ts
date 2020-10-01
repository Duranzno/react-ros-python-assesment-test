// // import { useContext, useState, useEffect } from "react"
// // import ROSLIB from 'roslib'
// // import { debounce } from 'ts-debounce';
// // eslint-disable-next-line @typescript-eslint/explicit-function-return-type 
// export function useTopicSubscribe()
// // <T>(name: string, messageType: string, equalityCheck: (a: any, b: any) => boolean) 
// {
//     // const { ros } = useContext(ROSContext)
//     // const [data, setData] = useState<T>()
//     // // const subs = new ROSLIB.Topic({ ros, name, messageType })
//     // // const topicSubscriptionCallback = (_message: Message): void => {
//     // //     const message = _message as T
//     // //     if (data === undefined) {
//     // //         console.log("Initial Data")
//     // //         setData(message)
//     // //     }
//     // //     else if (equalityCheck(message, data)) {
//     // //         console.log(`${JSON.stringify(message)} is updated`)
//     // //         setData(message)
//     // //     } else {
//     // //         console.log("DONT CHANGE")
//     // //     }
//     // // }
//     // useEffect(() => {
//     //     const f = debounce(console.log, 1000)
//     //     subs.subscribe((v) => { console.log(v); f(v) })
//     // }, [])

//     // const unsubscribe = (): void => subs.unsubscribe()
//     // return { data, unsubscribe }
// }
export { }