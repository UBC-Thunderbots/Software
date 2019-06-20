/***
 * This file facilitates the detection of the current browser to create browser
 * specific logic (ugh why does it have to be like this).
 */

import { detect } from 'detect-browser';
const browser = detect();

export const isChrome = browser!.name === 'chrome';
export const isFirefox = browser!.name === 'firefox';
