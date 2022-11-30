#pragma once
#if defined( KVS_SUPPORT_MPI )
#include <InSituVis/Lib/CameraPathControlledAdaptor_mpi.h>


namespace local
{

namespace mpi
{

class CameraFocusControlledAdaptor : public InSituVis::mpi::CameraPathControlledAdaptor
{
public:
    using BaseClass = InSituVis::mpi::CameraPathControlledAdaptor;
    using Controller = BaseClass::Controller;
    using FrameBuffer = BaseClass::FrameBuffer;
    using Viewpoint = InSituVis::Viewpoint;
    using Location = Viewpoint::Location;

private:
    kvs::Vec2i m_ndivs{ 20, 20 }; ///< number of divisions for frame buffer

public:
    CameraFocusControlledAdaptor( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root ) {}
    virtual ~CameraFocusControlledAdaptor() = default;

    void setNumberOfDivisions( const kvs::Vec2i& ndivs ) { m_ndivs = ndivs; }

protected:
    virtual void execRendering();

private:
    kvs::Vec3 look_at_in_window( const FrameBuffer& frame_buffer );
    kvs::Vec3 window_to_object( const kvs::Vec3 win, const Location& location );
    Location update_location( const Location& location, const kvs::Vec3 at );

    void crop_frame_buffer(
        const FrameBuffer& frame_buffer,
        const kvs::Vec2i& indices,
        FrameBuffer* cropped_frame_buffer );
};

} // end of namespace mpi

} // end of namespace local

#include "CameraFocusControlledAdaptor_mpi.hpp"

#endif // KVS_SUPPORT_MPI
