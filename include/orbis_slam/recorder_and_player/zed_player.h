#ifndef _ZED_PLAYER_H_
#define _ZED_PLAYER_H_

namespace Orbis {

class ZED_Player {
  public:
    ZED_Player();
    ~ZED_Player();

    void play(const unsigned long long& start_frame_idx = 0, const unsigned long long& );

    void pause();

    void step();

    void rewind();

    void fast_forward();

    void set_playback_speed(float speed);

    void jump_to_frame(unsigned long long frame_number);

    void jump_to_timestamp(double timestamp);

    void stop();
};  /* class ZED_Player */

} /* namespace Orbis */

#endif /* _ZED_PLAYER_H_ */