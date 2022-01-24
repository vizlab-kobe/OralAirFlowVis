/*****************************************************************************/
/**
 *  @file   EntropyControlledAdaptor_mpi.h
 *  @author Ken Iwata, Naohisa Sakamoto
 */
/*****************************************************************************/
#pragma once
#if defined( KVS_SUPPORT_MPI )
#include <InSituVis/Lib/Adaptor_mpi.h>
#include "EntropyTimestepController.h"
#include <list>
#include <queue>


namespace local
{

namespace mpi
{

class EntropyControlledAdaptor : public InSituVis::mpi::Adaptor, public local::EntropyTimestepController
{
public:
    using BaseClass = InSituVis::mpi::Adaptor;
    using Controller = local::EntropyTimestepController;

private:
    bool m_enable_output_image_depth = false;
    bool m_enable_output_evaluation_image = false; ///< if true, all of evaluation images will be output
    bool m_enable_output_evaluation_image_depth = false; ///< if true, all of evaluation depth images will be output
    kvs::mpi::StampTimer m_entr_timer{ BaseClass::world() }; ///< timer for entropy evaluation

public:
    EntropyControlledAdaptor( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root ) {}
    virtual ~EntropyControlledAdaptor() = default;

    void setOutputEvaluationImageEnabled(
        const bool enable = true,
        const bool enable_depth = false );

    virtual void exec( const BaseClass::SimTime sim_time = {} );
    virtual bool dump();

protected:
    bool isEntropyStep();
    virtual void execRendering();

private:
    kvs::Vec3 process( const Data& data );
    void process( const Data& data , const InSituVis::Viewpoint& path, const size_t i );

    void output_color_image(
        const InSituVis::Viewpoint::Location& location,
        const BaseClass::FrameBuffer& frame_buffer );

    void output_depth_image(
        const InSituVis::Viewpoint::Location& location,
        const BaseClass::FrameBuffer& frame_buffer );
    
    void output_heatmap(
        const size_t num_x,
        const size_t num_y,
        const std::vector<float>& entropy );
};

} // end of namespace mpi

} // end of namespace local

#include "EntropyControlledAdaptor_mpi.hpp"

#endif // KVS_SUPPORT_MPI
