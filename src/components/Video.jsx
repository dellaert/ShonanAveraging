import React from 'react';
import YouTube from 'react-youtube';

type Props = { videoId: string };

class Video extends React.Component<Props> {
  render() {
    const opts = {
      playerVars: {
        // https://developers.google.com/youtube/player_parameters
        autoplay: 0, controls: 1, enablejsapi: 1, fs: 1
      },
    };

    return <div className="embed-responsive embed-responsive-16by9">
      <YouTube className="embed-responsive-item"
        videoId={this.props.videoId} opts={opts}
        onReady={this._onReady}
      />
    </div>;
  }

  _onReady(event) {
    // access to player in all event handlers via event.target
    event.target.pauseVideo();
  }
}

export default Video;