let num = 10;

function add(a: number, b: number) {
    return a + b;
}

var s = add(num, num);

function logMessage(msg: string | null) {
    if (msg) {
        msg
    }
}

function resetOfPath(path: string) {
    return path
        .split('/')
        .splice(1)
        .join('/');
}