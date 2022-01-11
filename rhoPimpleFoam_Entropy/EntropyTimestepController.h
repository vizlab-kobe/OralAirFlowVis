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

    static float ColorEntropy( const FrameBuffer& frame_buffer );
    static float DepthEntropy( const FrameBuffer& frame_buffer );

private:
    size_t m_interval = 1; ///< time interval of entropy calculation

    bool m_cache_enabled = true; ///< flag for data caching
    size_t path_index = 0;
    size_t max_index = 0;
    DataQueue m_data_queue{}; ///< data queue
    Data m_previous_data{}; ///< dataset at previous time-step
    kvs::Vec3 m_previous_position; ///< best viewpoint position for the previous dataset
    kvs::Vec3 m_current_position; ///< best viewpoint position for the current dataset
    kvs::Vec3 m_previous_upVector;
    kvs::Vec3 m_current_upVector;
    InSituVis::Viewpoint m_path;
    EntropyFunction m_entropy_function = DepthEntropy;

public:
    EntropyTimestepController() = default;
    virtual ~EntropyTimestepController() = default;

    size_t entropyInterval() const { return m_interval; }
    InSituVis::Viewpoint path() const { return m_path; }
    size_t pathIndex() const { return path_index; }
    size_t maxIndex() const { return max_index; }
    kvs::Vec3 prvUpVector() const { return m_previous_upVector; }
    kvs::Vec3 crrUpVector() const { return m_current_upVector; }

    void setEntropyInterval( const size_t interval ) { m_interval = interval; }
    void setEntropyFunction( EntropyFunction func ) { m_entropy_function = func; }

    void setPath( const InSituVis::Viewpoint& path ) { m_path = path; }
    void setPathIndex( const size_t index ) { path_index = index; }
    void setMaxIndex( const size_t index ) { max_index = index; }
    void setPrvUpVector( const kvs::Vec3& upVector ) { m_previous_upVector = upVector; }
    void setCrrUpVector( const kvs::Vec3& upVector ) { m_current_upVector = upVector; }

    const DataQueue& dataQueue() const { return m_data_queue; }
    bool isCacheEnabled() const { return m_cache_enabled; }
    void setCacheEnabled( const bool enabled = true ) { m_cache_enabled = enabled; }

protected:
    void push( const Data& data );
    virtual kvs::Vec3 process( const Data& data ) {}
    virtual void process( const Data& data, const InSituVis::Viewpoint& path, const size_t i ) {}
    virtual float entropy( const FrameBuffer& frame_buffer );
};

} // end of namespace InSituVis

#include "EntropyTimestepController.hpp"
