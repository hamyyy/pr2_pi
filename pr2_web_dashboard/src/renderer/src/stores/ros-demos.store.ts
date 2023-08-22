import type rosnodejs from 'rosnodejs'
import type * as Shell from 'child_process'
import { get, writable, type Writable } from "svelte/store";

const ros: typeof rosnodejs = (window as any).api.ros
const shell: typeof Shell = (window as any).api.shell

export type ROSDemo = {
    name: string
    file: string
    machine: 'c1' | 'c2'
    state: 'running' | 'stopped' | 'starting' | 'stopping',
    pid?: number
}

type ROSLaunchPID = {
    name: string
    file: string
    machine: 'c1' | 'c2'
    pid: number
}

export let demos: Writable<ROSDemo[]> = writable([]);

export async function scanForDemos() {
    const demoArray: ROSDemo[] = [];
    const pidArray: ROSLaunchPID[] = [];

    let output = "";

    try {
        output = shell.execSync(`ssh pr2@c1 "ls -1A /home/pr2/pr2pi_shared/"`).toString();
    } catch (e) {
        console.error(e);
        return;
    }

    {
        const lines = output.split("\n");
        for (let line of lines) {
            line = line.trim();
            if (line === "") continue;
            if (line.endsWith(".launch")) {
                const name = line.replace(".launch", "").replace(/[-_]/g, " ");
                demoArray.push({
                    name,
                    file: `/home/pr2/pr2pi_shared/${line}`,
                    machine: 'c1',
                    state: 'stopped',
                })
            } else if (line.endsWith(".launch.pid")) {
                const name = line.replace(".launch.pid", "").replace(/[-_]/g, " ");
                const file = `/home/pr2/pr2pi_shared/${line}`;

                try {
                    let pidOutput = shell.execSync(`ssh pr2@c1 "cat ${file}"`).toString();
                    const pid = parseInt(pidOutput.trim());

                    if (isNaN(pid)) {
                        console.error(`Invalid PID in file ${file}`);
                    } else {
                        pidArray.push({
                            name,
                            file,
                            machine: 'c1',
                            pid
                        })
                    }

                } catch (e) {
                    console.error(e);
                }
            }
        }
    }

    output = "";
    try {
        output = shell.execSync(`ssh pr2@c2 "ls -1A /home/pr2/pr2pi_shared/"`).toString();
    } catch (e) {
        console.error(e);
        return;
    }

    {
        const lines = output.split("\n");
        for (let line of lines) {
            line = line.trim();
            if (line === "") continue;
            if (line.endsWith(".launch")) {
                const name = line.replace(".launch", "").replace(/[-_]/g, " ");
                demoArray.push({
                    name,
                    file: `/home/pr2/pr2pi_shared/${line}`,
                    machine: 'c2',
                    state: 'stopped',
                })
            } else if (line.endsWith(".launch.pid")) {
                const name = line.replace(".launch.pid", "").replace(/[-_]/g, " ");
                const file = `/home/pr2/pr2pi_shared/${line}`;

                try {
                    let pidOutput = shell.execSync(`ssh pr2@c2 "cat ${file}"`).toString();
                    const pid = parseInt(pidOutput.trim());

                    if (isNaN(pid)) {
                        console.error(`Invalid PID in file ${file}`);
                    } else {
                        pidArray.push({
                            name,
                            file,
                            machine: 'c2',
                            pid
                        })
                    }

                } catch (e) {
                    console.error(e);
                }
            }
        }
    }


    checkPIDs(demoArray, pidArray);
    demos.set(demoArray);
}


export function checkPIDs(demoArray: ROSDemo[], pidArray: ROSLaunchPID[]) {
    for (let i = pidArray.length - 1; i >= 0; i--) {

        let demo = demoArray.find(demo => demo.name === pidArray[i].name && demo.machine === pidArray[i].machine)
        if (!demo) {
            shell.execSync(`rm ${pidArray[i].file}`);
            pidArray.splice(i, 1);
            continue;
        }

        let filename = demo.file.split("/").pop();
        try {
            let output = shell.execSync(`ssh pr2@${demo.machine} "ps aux --no-headers -q ${pidArray[i].pid} | grep ${filename}"`).toString();


            if (output.trim() === "") {
                shell.execSync(`ssh pr2@${demo.machine} "rm ${pidArray[i].file}"`);
                pidArray.splice(i, 1);
                continue;
            }

            demo.state = 'running';
            demo.pid = pidArray[i].pid;
        } catch (e) {
            shell.execSync(`ssh pr2@${demo.machine} "rm ${pidArray[i].file}"`);
            pidArray.splice(i, 1);
            continue;
        }
    }
}

export function runDemo(demo: ROSDemo) {

    switch (demo.state) {
        case "running":
            throw new Error(`Demo "${demo.name}" is already running`);
        case "starting":
            throw new Error(`Demo "${demo.name}" is already starting`);
        case "stopping":
            throw new Error(`Demo "${demo.name}" is stopping, please wait`);
    }

    demo.state = 'starting';
    demos.update(d => d);

    const roscmd = `roslaunch ${demo.file}`;

    const cmd = `ssh pr2@${demo.machine} "source ~/.profile && /home/pr2/pr2pi_shared/run.sh ${roscmd}"`;
    console.log(`Running command:\n${cmd}`);
    shell.exec(cmd, (error, stdout, stderr) => {
        setTimeout(() => scanForDemos());

        if (error) {
            console.error(error);
            if (stdout) console.log(stdout);
            if (stderr) console.error(stderr);
            demo.state = 'stopped';
            demos.update(d => d);
            return;
        }

        console.log("Started:", demo.name);
        console.log(stdout);
    });

}

export function stopDemo(demo: ROSDemo) {

    switch (demo.state) {
        case "stopped":
            throw new Error(`Demo "${demo.name}" is already stopped`);
        case "starting":
            throw new Error(`Demo "${demo.name}" is starting, please wait`);
    }

    demo.state = 'stopping';
    demos.update(d => d);

    const cmd = `ssh pr2@${demo.machine} "kill ${demo.pid}"`;

    shell.exec(cmd, (error) => {
        setTimeout(() => scanForDemos());
        if (error) {
            console.error(error);
            return;
        }

        demo.state = 'stopped';
        demos.update(d => d);
    });
}


export function stopAllDemos() {
    const ds = get(demos);
    for (const d of ds) {
        try {
            stopDemo(d);

        }
        catch (e) { }
    }
}