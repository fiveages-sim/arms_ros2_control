//
// Read Frequency Controller - Utility for controlling read frequency
//
#pragma once

namespace gripper_hardware_common
{
    /**
     * @brief Utility class for controlling read frequency
     * 
     * This class helps reduce Modbus communication load by controlling
     * how frequently status is read from grippers.
     */
    class ReadFrequencyController
    {
    public:
        /**
         * @brief Default read interval (read every 4 cycles)
         */
        static constexpr int DEFAULT_INTERVAL = 4;

        /**
         * @brief Constructor
         * @param interval Read interval (read every N cycles, default: 4)
         */
        explicit ReadFrequencyController(int interval = DEFAULT_INTERVAL)
            : interval_(interval)
            , counter_(0)
        {
        }

        /**
         * @brief Check if it's time to read
         * 
         * Increments internal counter and returns true when counter reaches interval.
         * 
         * @return true if it's time to read, false otherwise
         */
        bool shouldRead()
        {
            counter_++;
            if (counter_ >= interval_)
            {
                counter_ = 0;
                return true;
            }
            return false;
        }

        /**
         * @brief Reset the counter
         */
        void reset()
        {
            counter_ = 0;
        }

        /**
         * @brief Get current counter value
         * @return Current counter value
         */
        int getCounter() const
        {
            return counter_;
        }

        /**
         * @brief Get read interval
         * @return Read interval
         */
        int getInterval() const
        {
            return interval_;
        }

        /**
         * @brief Set read interval
         * @param interval New read interval
         */
        void setInterval(int interval)
        {
            interval_ = interval;
            reset();
        }

    private:
        int interval_;   // Read interval (read every N cycles)
        int counter_;    // Current counter value
    };
} // namespace gripper_hardware_common
