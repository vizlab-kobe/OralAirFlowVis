#pragma once
#if defined( KVS_SUPPORT_MPI )
#include <InSituVis/Lib/Adaptor_mpi.h>
#include <kvs/UnstructuredVolumeObject>


namespace local
{

namespace mpi
{

class PointFocusAdaptor : public InSituVis::mpi::Adaptor
{
public:
    using BaseClass = InSituVis::mpi::Adaptor;
    using Volume = kvs::UnstructuredVolumeObject;

public:
    PointFocusAdaptor( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root ) {}
    virtual ~PointFocusAdaptor() = default;

protected:
    virtual void execRendering();

private:
    kvs::Vec3 position_with_max_value();
    void update_viewpoint( const kvs::Vec3& at );
};

} // end of namespace mpi

} // end of namespace local

#include "PointFocusAdaptor_mpi.hpp"

#endif // KVS_SUPPORT_MPI
