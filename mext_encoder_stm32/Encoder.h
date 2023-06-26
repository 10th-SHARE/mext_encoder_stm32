#ifndef ENCODER_H_INCLUDED
#define ENCODER_H_INCLUDED
 
#include "encoder_mbed.h"
#include <mstd_atomic>
#include "mbed.h"
 
namespace mext {

    class Qei {
        public:
        explicit Qei(PinName pin1, PinName pin2)
                : revert_(false)
            {
                if (mhal::encoder_pin_polarity(pin1, pin2) == mhal::encoder_pin_polarity_rev) {
                    mhal::encoder_init(&enc_, pin2, pin1);
                    revert_ = true;
                } else { // polarity_syn or polarity_none
                    mhal::encoder_init(&enc_, pin1, pin2);
                }
                resol_ = mhal::encoder_counter_resolution(&enc_);
                write_zero();
            }

            int counter() const {

                std::int32_t cnt;

                if (resol_ == mhal::encoder_counter_resolution_32bit) {
                    cnt = static_cast<std::int32_t>(mhal::encoder_read(&enc_));
                } else {
                    cnt = static_cast<std::int16_t>(mhal::encoder_read(&enc_));
                }

                if (revert_) {
                    cnt = -cnt;
                }

                return static_cast<int>(cnt);
            }

            void write_zero() {
                mhal::encoder_write(&enc_, 0);
            }
            
            ~Qei() {
                mhal::encoder_free(&enc_);
            }

        private:
            DeepSleepLock lock;
            mhal::encoder_t enc_;
            mhal::encoder_counter_resolution_t resol_;
            bool revert_; 
    };
    
    template <class T = int>
    class Encoder {
        public:
            static_assert(std::is_integral<T>::value, "T must be integral");

            explicit Encoder(PinName pin1, PinName pin2, int offset = 0)
                : accumulated_ticks_(offset), qei_(pin1, pin2)
            {
            }

            int counter() const {
                return qei_.counter();
            }
        
            T ticks() const {
                CriticalSectionLock lock;
                return qei_.counter() + accumulated_ticks_;
            }
        
            void accumulate() {
                CriticalSectionLock lock;
        
                accumulated_ticks_ += qei_.counter();
                qei_.write_zero();
            }
        
            void reset(int offset = 0) {
                CriticalSectionLock lock;

                accumulated_ticks_ = offset;
                qei_.write_zero();
            }
        
        private:
            T accumulated_ticks_;
            Qei qei_;
    };

    class Ec : public Encoder<int> {
        public:
            explicit Ec(PinName pin1, PinName pin2, int res)
                : Encoder<int>(pin1, pin2, 0), resolution_(res * multiplication_)
            {
                setGearRatio(1); //ギア比1(減速なし)
                omega_ = 0.0;      
                pre_omega_ = 0.0;  
                pre2_omega_ = 0.0; 
                acceleration_ = 0.0;
                ptw_ = 0.0;
                timer_.start(); //タイマー始める
            }

            //エンコーダのカウント数を返す関数 1周のカウント数=分解能×逓倍
            int getCount() const {
                return ticks();
            }

            //軸の回転角度を返す関数 [rad] 
            double getRad() const {
                //カウント数 × 2π / (分解能 * ギア比)
                return ticks() * 2.0f * M_PI / (resolution_ * gear_ratio_);
            }

            //軸の回転角度を返す関数 [度]
            double getDeg() const {
                return ticks() * 2.0f * 180.0f / (resolution_ * gear_ratio_);
            }

            //軸の回転速度・角加速度を計算するための関数 
            //微分を微小時間の変位として計算しているためタイマー割り込みなどで回す
            void calOmega() {
                if(ptw_ == 0.0) {
                    ptw_ = timer_.read();
                } else {
                    double t = timer_.read(); //タイマー読み込み
                    timer_.stop();
                    timer_.reset();
                    timer_.start();
                    acceleration_ = (pre_omega_ - pre2_omega_) / t; //角速度の微小時間での変位
                    pre2_omega_ = pre_omega_;
                    pre_omega_ = omega_;
                    //カウント数 * 2π / (分解能 * ギア比 * 微小時間)
                    omega_ = counter() * 2.0f * M_PI / (resolution_ * gear_ratio_ * t);
                    accumulate();
                    ptw_ = t;
                }
            }

            //軸の回転速度を返す関数 [rad/s]
            double getOmega() const {
                return omega_;
            }

            //軸の角加速度を返す関数 [rad/s^2]
            double getAcceleration() const {
                return acceleration_;
            }
            
            //ギア比を設定する関数 デフォルトの値は1(減速無し)
            void setGearRatio(double gear_r) {
                gear_ratio_ = gear_r;
            }
 

        private:
            Timer timer_;
            static constexpr double M_PI = 3.14159265359f;
            static constexpr int multiplication_ = 4; 
            double omega_;        //角速度(rad/s)
            double pre_omega_;    //前回の角速度(rad/s)
            double pre2_omega_;   //前々回の角速度(rad/s)
            double acceleration_; //角加速度
            int resolution_;      //分解能 * 逓倍
            double ptw_;          //前回のタイマー時間
            double gear_ratio_;   //ギア比（タイヤ:エンコーダ＝１:rとしたときのrの値）
    };
}
 
#endif
