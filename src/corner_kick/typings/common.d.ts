/**
 * This file defines additional extensions that could be present in Typescipt code and
 * are supported by our Webpack configuration. Without defining them, Typescript will
 * throw a transpile time error when we try to import a file of this type.
 */

declare module '*.css';
declare module '*.png';
declare module '*.jpg';
declare module '*.svg';
