import { detect } from 'detect-browser';
const browser = detect();

export const isChrome = browser!.name === 'chrome';
export const isFirefox = browser!.name === 'firefox';
