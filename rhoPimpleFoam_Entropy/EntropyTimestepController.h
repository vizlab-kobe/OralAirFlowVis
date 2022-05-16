/*****************************************************************************/
/**
 *  @file   EntropyTimestepController.h
 *  @author Ken Iwata, Naohisa Sakamoto
 */
/*****************************************************************************/
#pragma once
#include <queue>
#include <functional>
#include <kvs/VolumeObjectBase>
#include <InSituVis/Lib/Adaptor_mpi.h>
#include <InSituVis/Lib/Viewpoint.h>


namespace local
{

class EntropyTimestepController
{
public:
    using BaseClass = InSituVis::mpi::Adaptor;
    using Data = InSituVis::Adaptor::ObjectList;
    using DataQueue = std::queue<Data>;

    using Volume = kvs::VolumeObjectBase;
    using Values = Volume::Values;

    using FrameBuffer = InSituVis::mpi::Adaptor::FrameBuffer;
    using EntropyFunction = std::function<float(const FrameBuffer&)>;

    static float Entropy( const FrameBuffer& frame_buffer );
    static float ColorEntropy( const FrameBuffer& frame_buffer );
    static float DepthEntropy( const FrameBuffer& frame_buffer );

private:
    size_t m_interval = 1; ///< time interval of entropy calculation
    kvs::Vec3 m_position0{ 0.0f, 12.0f, 0.0f };

    bool m_cache_enabled = true; ///< flag for data caching
    size_t m_path_index = 0;
    size_t m_max_index = 0;
    kvs::Vec3 m_max_position;
    kvs::Vec3 m_max_up_vector;
    DataQueue m_data_queue{}; ///< data queue
    Data m_previous_data{}; ///< dataset at previous time-step
    size_t m_previous_index;
    size_t m_current_index;
    kvs::Vec3 m_previous_position; ///< best viewpoint position for the previous dataset
    kvs::Vec3 m_current_position; ///< best viewpoint position for the current dataset
    InSituVis::Viewpoint m_path;
    size_t m_pole_number = 0;
    kvs::Vec3 m_pole_position;
    EntropyFunction m_entropy_function = Entropy;
    std::queue<kvs::Quaternion> m_max_positions;
    size_t count = 0;

public:
    EntropyTimestepController() = default;
    virtual ~EntropyTimestepController() = default;

    size_t entropyInterval() const { return m_interval; }
    size_t pathIndex() const { return m_path_index; }
    size_t maxIndex() const { return m_max_index; }
    kvs::Vec3 maxPosition() const { return m_max_position; }
    kvs::Vec3 maxUpVector() const { return m_max_up_vector; }
    size_t previousIndex() const { return m_previous_index; }
    size_t currentIndex() const { return m_current_index; }
    kvs::Vec3 previousPosition() const { return m_previous_position; }
    kvs::Vec3 currentPosition() const { return m_current_position; }
    InSituVis::Viewpoint path() const { return m_path; }
    size_t poleNumber() const { return m_pole_number; }
    kvs::Vec3 polePosition() const { return m_pole_position; }

    void setEntropyInterval( const size_t interval ) { m_interval = interval; }
    void setEntropyFunction( EntropyFunction func ) { m_entropy_function = func; }

    void setPathIndex( const size_t index ) { m_path_index = index; }
    void setMaxIndex( const size_t index ) { m_max_index = index; }
    void setMaxPosition( const kvs::Vec3& position ) { m_max_position = position; }
    void setMaxUpVector( const kvs::Vec3& up_vector ) { m_max_up_vector = up_vector; }
    void setPreviousIndex( const size_t index ) { m_previous_index = index; }
    void setCurrentIndex( const size_t index ) { m_current_index = index; }
    void setPreviousPosition( const kvs::Vec3& position ) { m_previous_position = position; }
    void setCurrentPosition( const kvs::Vec3& position ) { m_current_position = position; }
    void setPath( const InSituVis::Viewpoint& path ) { m_path = path; }
    void setPoleNumber( const size_t pole_num ) { m_pole_number = pole_num; }
    void setPolePosition( const kvs::Vec3& pole_position ) { m_pole_position = pole_position; }

    const DataQueue& dataQueue() const { return m_data_queue; }
    const Data& previousData() const { return m_previous_data; }
    bool isCacheEnabled() const { return m_cache_enabled; }
    void setCacheEnabled( const bool enabled = true ) { m_cache_enabled = enabled; }

protected:
    void push( const Data& data );
    virtual void process( const Data& data ) {}
    virtual void process( const Data& data, const size_t i ) {}
    virtual float entropy( const FrameBuffer& frame_buffer );
    InSituVis::Viewpoint createPathSpline(
        const kvs::Quaternion& q1,
        const kvs::Quaternion& q2,
        const kvs::Quaternion& q3,
        const kvs::Quaternion& q4,
        const size_t point_interval
    );
};

} // end of namespace InSituVis

#include "EntropyTimestepController.hpp"
