// @flow
// Component for paper landing site, pointing to paper, video, and code, resp.
// React component by Frank Dellaert
// Adapted from https://people.eecs.berkeley.edu/~pratul/lighthouse/
// In turn adapted from http://mgharbi.com/
import React from 'react';
import Image from 'react-bootstrap/Image';
import Nav from 'react-bootstrap/Nav';

type Props = { paper: string; video: string; repo: string };

function Media(props: Props) {
    return (
        <div className="row">
            <div className="col" />
            <div className="col-9 text-center">
                <Nav variant="pills" justify>
                    <Nav.Item key="paper">
                        <a href="{props.paper}">
                            <img src="assets/images/paper.jpg" height="120px" /><br />
                            <h4><strong>Paper</strong></h4>
                        </a>
                    </Nav.Item>
                    <Nav.Item key="video">
                        <a href="{props.video}">
                            <img src="assets/images/youtube_icon_dark.png" height="120px" /><br />
                            <h4><strong>Technical Video</strong></h4>
                        </a>
                    </Nav.Item>
                    <Nav.Item key="repo">
                        <a href="{props.repo}">
                            <img src="assets/images/github_pad.png" height="120px" /><br />
                            <h4><strong>Code</strong></h4>
                        </a>
                    </Nav.Item>
                </Nav>
            </div>
            <div className="col" />
        </div>
    );
}

export default Media;
