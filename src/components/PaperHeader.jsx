// @flow
// Component for paper landing site, pointing to paper, video, and code, resp.
// React component by Frank Dellaert
// Adapted from https://people.eecs.berkeley.edu/~pratul/lighthouse/
// In turn adapted from http://mgharbi.com/
import React from 'react';
import Image from 'react-bootstrap/Image';
import Nav from 'react-bootstrap/Nav';

// Not affiliation is base 1
type AuthorRecord = { name: string; affiliation: number; link?: string };

type Props = {
    title: string; subtitle?: string; venue: string;
    authors: Array<AuthorRecord>;
    affiliations: Array<string>;
};

function Author(props: AuthorRecord) {
    const { name, affiliation, link } = props;
    return (<li className="list-inline-item" key={name}>
        {link
            ? (<a href={link}>{name}<sup>{affiliation}</sup></a>)
            : (<span>{name}<sup>{affiliation}</sup></span>)
        }
    </li>);
}

function PaperHeader(props: Props) {
    return (
        <div>
            <div className="row">
                <h1 className="col text-center">{props.title}</h1>
            </div>
            <div className="row">
                <h2 className="col text-center">{props.subtitle}</h2>
            </div>
            <div className="row">
                <h4 className="col text-center">{props.venue}</h4>
            </div>
            <div className="row">
                <div className="col text-center">
                    <ul className="list-inline">
                        {props.authors.map(author => <Author {...author} />)}
                    </ul>
                </div>
            </div>
            <div className="row">
                <div className="col text-center">
                    <ul className="list-inline">
                        {props.affiliations.map((name, index) =>
                            <li className="list-inline-item" key={index}><sup>{index + 1}</sup>{name}</li>
                        )}
                    </ul>
                </div>
            </div>
            {/* <div className="row">
                <div className="col text-center">
                    <div className="col text-center">
                        *denotes equal contribution
                        </div>
                </div>
            </div> */}
        </div>
    );
}

export default PaperHeader;
