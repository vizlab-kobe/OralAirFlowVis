#include <InSituVis/Lib/Viewpoint.h>
#include <kvs/Quaternion>
#include <kvs/Coordinate>
#include <kvs/ObjectManager>


namespace local
{

namespace mpi
{

inline void PointFocusAdaptor::execRendering()
{
    auto at = this->position_with_max_value();
    this->update_viewpoint( at );

    BaseClass::execRendering();
}

inline kvs::Vec3 PointFocusAdaptor::position_with_max_value()
{
    kvs::Real32 max_value = kvs::Value<kvs::Real32>::Min();
    kvs::Vec3 max_coord( 0, 0, 0 );
    for ( auto& object : BaseClass::objects() )
    {
        auto* volume = Volume::DownCast( object.get() );
        if ( !volume ) { continue; }
        if ( volume->veclen() != 1 ) { continue; }
        if ( volume->connections().empty() ) { continue; }

        const auto* indices = volume->connections().data();
        const auto values = volume->values().asValueArray<kvs::Real32>();
        const auto coords = volume->coords();
        const auto ncells = volume->numberOfCells();
        const auto cell_nnodes = volume->numberOfCellNodes();

        for ( size_t i = 0; i < ncells; i++ )
        {
            for ( size_t j = 0; j < cell_nnodes; ++j )
            {
                const auto index = *indices++;
                const auto value = values[ index ];
                if ( value > max_value )
                {
                    max_value = value;
                    max_coord = {
                        coords[ 3 * index ],
                        coords[ 3 * index + 1 ],
                        coords[ 3 * index + 2 ] };
                }
            }
        }
    }

    kvs::ValueArray<kvs::Real32> max_values;
    BaseClass::world().allGather( max_value, max_values );

    auto max_rank = max_values.argmax();
    BaseClass::world().broadcast( max_rank, max_coord.data(), 3 );

    const auto* base = BaseClass::screen().scene()->objectManager()->object();
    return kvs::ObjectCoordinate( max_coord, base ).toWorldCoordinate().position();
}

inline void PointFocusAdaptor::update_viewpoint( const kvs::Vec3& at )
{
    InSituVis::Viewpoint vp;
    for ( const auto& location : BaseClass::viewpoint().locations() )
    {
        const auto p0 = location.look_at - location.position;
        const auto p1 = at - location.position;
        const auto R = kvs::Quat::RotationQuaternion( p0, p1 );

        auto l = InSituVis::Viewpoint::Location(
            location.direction,
            location.position,
            kvs::Quat::Rotate( location.up_vector, R ), at );
        l.index = location.index;
        l.look_at = at;

        vp.add( l );
    }
    BaseClass::setViewpoint( vp );
}

} // end of namespace mpi

} // end of namespace local
