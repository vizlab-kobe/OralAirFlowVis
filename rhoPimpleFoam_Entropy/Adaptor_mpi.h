/*****************************************************************************/
/**
 *  @file   Adaptor_mpi.h
 *  @author Naohisa Sakamoto
 */
/*****************************************************************************/
#pragma once
#include <InSituVis/Lib/Adaptor.h>
#if defined( KVS_SUPPORT_MPI )
#include <kvs/mpi/Communicator>
#include <kvs/mpi/LogStream>
#include <kvs/mpi/ImageCompositor>
#include <kvs/mpi/StampTimer>


namespace local
{

namespace mpi
{

/*===========================================================================*/
/**
 *  @brief  Adaptor class for parallel rendering based on MPI.
 */
/*===========================================================================*/
class Adaptor : public InSituVis::Adaptor
{
public:
    using BaseClass = InSituVis::Adaptor;
    using DepthBuffer = kvs::ValueArray<kvs::Real32>;

    struct FrameBuffer
    {
        ColorBuffer color_buffer;
        DepthBuffer depth_buffer;
        FrameBuffer() = default;
        FrameBuffer( const ColorBuffer& cb, const DepthBuffer& db ):
            color_buffer( cb ),
            depth_buffer( db ) {}
        FrameBuffer( const size_t width, const size_t height ):
            color_buffer( width * height * 4 ),
            depth_buffer( width * height ) {}
    };

private:
    kvs::mpi::Communicator m_world{}; ///< MPI communicator
    kvs::mpi::LogStream m_log{ m_world }; ///< MPI log stream
    kvs::mpi::ImageCompositor m_image_compositor{ m_world }; ///< image compositor
    bool m_enable_output_subimage = false; ///< flag for writing sub-object rendering image
    bool m_enable_output_subimage_depth = false; ///< flag for writing sub-object rendering image (depth image)
    bool m_enable_output_subimage_alpha = false; ///< flag for writing sub-object rendering image (alpha image)
    float m_rend_time = 0.0f; ///< rendering time per frame
    float m_comp_time = 0.0f; ///< image composition time per frame
    kvs::mpi::StampTimer m_comp_timer{ m_world }; ///< timer for image composition process

public:
    Adaptor( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): m_world( world, root ) {}
    virtual ~Adaptor() = default;

    kvs::mpi::Communicator& world() { return m_world; }
    std::ostream& log() { return m_log( m_world.root() ); }
    std::ostream& log( const int rank ) { return m_log( rank ); }
    kvs::StampTimer& compTimer() { return m_comp_timer; }

    void setOutputSubImageEnabled(
        const bool enable = true,
        const bool enable_depth = false,
        const bool enable_alpha = false );

    virtual bool initialize();
    virtual bool finalize();
    virtual void exec( const BaseClass::SimTime sim_time = {} );
    virtual bool dump();

protected:
    virtual void execRendering();
    virtual FrameBuffer drawScreen( std::function<void(const FrameBuffer&)> func = [] ( const FrameBuffer& ) {} );

    float rendTime() const { return m_rend_time; }
    float compTime() const { return m_comp_time; }
    void setRendTime( const float time ) { m_rend_time = time; }
    void setCompTime( const float time ) { m_comp_time = time; }

    kvs::mpi::ImageCompositor& imageCompositor() { return m_image_compositor; }
    std::string outputFinalImageName( const InSituVis::Viewpoint::Location& location );
    void outputSubImages(
        const FrameBuffer& frame_buffer,
        const InSituVis::Viewpoint::Location& location,
        const std::string& suffix = "" );

    DepthBuffer backgroundDepthBuffer();
    FrameBuffer readback( const InSituVis::Viewpoint::Location& location );

private:
    FrameBuffer readback_uni_buffer( const InSituVis::Viewpoint::Location& location );
    FrameBuffer readback_omn_buffer( const InSituVis::Viewpoint::Location& location );
    FrameBuffer readback_adp_buffer( const InSituVis::Viewpoint::Location& location );
};

} // end of namespace mpi

} // end of namespace InSituVis

#include "Adaptor_mpi.hpp"

#endif // KVS_SUPPORT_MPI
