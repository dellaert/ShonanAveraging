// @flow
// Component for paper landing site, pointing to paper, video, and code, resp.
// React component by Frank Dellaert
// Adapted from https://people.eecs.berkeley.edu/~pratul/lighthouse/
// In turn adapted from http://mgharbi.com/
import React from 'react';
import Image from 'react-bootstrap/Image';

type Props = { paper: string; video:string; repo:string};

function Media(props: Props) {
    return (
        <div class="row">
            <div class="col-md-8 col-md-offset-2 text-center">
                <ul class="nav nav-pills nav-justified">
                    <li>
                        <a href="{props.paper}">
                            <image src="assets/images/paper.jpg" height="120px" /><br />
                            <h4><strong>Paper</strong></h4>
                        </a>
                    </li>
                    <li>
                        <a href="{props.video}">
                            <image src="assets/images/youtube_icon_dark.png" height="120px" /><br />
                            <h4><strong>Technical Video</strong></h4>
                        </a>
                    </li>
                    <li>
                        <a href="{props.repo}">
                            <image src="assets/images/github_pad.png" height="120px" /><br />
                            <h4><strong>Code</strong></h4>
                        </a>
                    </li>
                </ul>
            </div>
        </div>
    );
}

export default Media;
