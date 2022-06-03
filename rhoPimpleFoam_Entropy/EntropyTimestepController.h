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

    bool m_cache_enabled = true; ///< flag for data caching
    size_t m_path_index = 0;
    size_t m_max_index = 0;
    kvs::Vec3 m_max_position;
    kvs::Quaternion m_max_rotation;
    kvs::Quaternion m_erp_rotation;
    DataQueue m_data_queue{}; ///< data queue
    Data m_previous_data{}; ///< dataset at previous time-step
    std::queue<kvs::Quaternion> m_path;
    EntropyFunction m_entropy_function = Entropy;
    std::queue<kvs::Quaternion> m_max_rotations;
    size_t count = 0;

public:
    EntropyTimestepController() = default;
    virtual ~EntropyTimestepController() = default;

    size_t entropyInterval() const { return m_interval; }
    size_t pathIndex() const { return m_path_index; }
    size_t maxIndex() const { return m_max_index; }
    kvs::Vec3 maxPosition() const { return m_max_position; }
    kvs::Quaternion maxRotation() const { return m_max_rotation; }
    kvs::Quaternion erpRotation() const { return m_erp_rotation; }

    void setEntropyInterval( const size_t interval ) { m_interval = interval; }
    void setEntropyFunction( EntropyFunction func ) { m_entropy_function = func; }

    void setMaxIndex( const size_t index ) { m_max_index = index; }
    void setMaxPosition( const kvs::Vec3& position ) { m_max_position = position; }
    void setMaxRotation( const kvs::Quaternion& rotation ) { m_max_rotation = rotation; }
    void setErpRotation( const kvs::Quaternion& rotation ) { m_erp_rotation = rotation; }

    const DataQueue& dataQueue() const { return m_data_queue; }
    const Data& previousData() const { return m_previous_data; }
    bool isCacheEnabled() const { return m_cache_enabled; }
    void setCacheEnabled( const bool enabled = true ) { m_cache_enabled = enabled; }

protected:
    void push( const Data& data );
    virtual void process( const Data& data ) {}
    virtual void process( const Data& data, const kvs::Quaternion& rotation ) {}
    virtual float entropy( const FrameBuffer& frame_buffer );
    void createPathSlerp(
        const kvs::Quaternion& q1,
        const kvs::Quaternion& q2,
        const size_t point_interval
    );
    void createPathSpline(
        const kvs::Quaternion& q1,
        const kvs::Quaternion& q2,
        const kvs::Quaternion& q3,
        const kvs::Quaternion& q4,
        const size_t point_interval
    );
};

} // end of namespace InSituVis

#include "EntropyTimestepController.hpp"
