/*****************************************************************************/
/**
 *  @file   AdaptiveTimestepController.h
 *  @author Naohisa Sakamoto
 */
/*****************************************************************************/
#pragma once
#include <queue>
#include <functional>
#include <kvs/VolumeObjectBase>
#include "Adaptor_mpi.h"


namespace local
{

class AdaptiveTimestepController
{
public:
    using BaseClass = local::mpi::Adaptor;
    using Data = InSituVis::Adaptor::ObjectList;
    using DataQueue = std::queue<Data>;

    using Volume = kvs::VolumeObjectBase;
    using Values = Volume::Values;

private:
    size_t m_interval = 1; ///< time interval of entropy calculation

    bool m_cache_enabled = true; ///< flag for data caching
    DataQueue m_data_queue{}; ///< data queue
    Data m_previous_data{}; ///< dataset at previous time-step
    kvs::Vec3 m_previous_position; ///< best viewpoint position for the previous dataset
    kvs::Vec3 m_current_position; ///< best viewpoint position for the current dataset
    InSituVis::Viewpoint m_path;

public:
    AdaptiveTimestepController() = default;
    virtual ~AdaptiveTimestepController() = default;

    size_t calculationInterval() const { return m_interval; }

    void setCalculationInterval( const size_t interval ) { m_interval = interval; }
    void setPath( const InSituVis::Viewpoint& path ) { m_path = path; }

    const DataQueue& dataQueue() const { return m_data_queue; }
    bool isCacheEnabled() const { return m_cache_enabled; }
    void setCacheEnabled( const bool enabled = true ) { m_cache_enabled = enabled; }

protected:
    void push( const Data& data );
    virtual kvs::Vec3 calcProcess( const Data& data ) {}
    virtual void process( const Data& data, const InSituVis::Viewpoint& path, const size_t i ) {}
};

} // end of namespace InSituVis

#include "AdaptiveTimestepController.hpp"
