import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

const quotes = [
    [
        'Some people think football [soccer]is a matter of life and death. ' +
            'I don’t like that attitude. I can assure them it is ' +
            'much more serious than that.',
        'Bill Shankly',
    ],
    [
        'Football is a simple game; 22 men chase a ball for ' +
            '90 minutes and at the end, the Germans win.',
        'Gary Lineker',
    ],
    [
        'We must have had 99 percent of the game. it was the other ' +
            'three percent that cost us the match.',
        'Ruud Gullit',
    ],
    ['I learned all about life with a ball at my feet', 'Ronaldinho'],
    ['The more difficult the victory, the greater the happiness in winning.', 'Pelé'],
    [
        'Good soccer players need not be titans sculpted by Michelangelo. In soccer, ' +
            'ability is much more important than shape, and in many cases, skill is' +
            ' the art of turning limitations into virtues.',
        'Eduardo Galeano',
    ],
    [
        'Becoming a footballer is only the first half of the silent prayer a kid offers' +
            ' up to the sky or confides to his teacher in a primary school essay. The' +
            ' second part is the name of the team he wants to play for.',
        'Andrea Pirlo',
    ],
    [
        'The ball is round, the game lasts ninety minutes, and everything else is just' +
            ' theory.',
        'Josef “Sepp” Herberger',
    ],
    [
        'When good soccer happens, I give thanks for the miracle and I don’t ' +
            'give a damn which team or country performs it.',
        'Eduardo Galeano',
    ],
    [
        'You have to fight to reach your dream. You have to sacrifice and work' +
            ' hard for it.',
        'Lionel Messi',
    ],
    [
        'In football, the worst blindness is only seeing the ball.',
        'Nelson Falcão Rodrigues',
    ],
];

const Wrapper = styled.div`
    width: 100%;
    height: 100%;

    display: flex;
    flex-flow: column nowrap;

    justify-content: center;
    align-items: center;

    padding: 16px;

    color: ${dark.syntax.comment};
`;

const Icon = styled.i`
    font-size: 48px;
`;

const Quote = styled.div`
    font-weight: 700;
    text-align: center;
    max-width: 768px;
`;

const Author = styled.div`
    font-style: italic;
    text-align: center;
`;

export const EmptyMain = () => {
    const i = Math.round(Math.random() * quotes.length);

    return (
        <Wrapper>
            <Icon className="material-icons">weekend</Icon>
            <Quote>{quotes[i][0]}</Quote>
            <Author>{quotes[i][1]}</Author>
        </Wrapper>
    );
};
