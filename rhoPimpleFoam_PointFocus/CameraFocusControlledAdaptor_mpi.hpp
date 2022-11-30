#include "CameraFocusControlledAdaptor_mpi.h"
#include <kvs/Assert>
#include <kvs/ObjectManager>


namespace local
{

namespace mpi
{

inline void CameraFocusControlledAdaptor::execRendering()
{
    BaseClass::setRendTime( 0.0f );
    BaseClass::setCompTime( 0.0f );
    float save_time = 0.0f;
    float entr_time = 0.0f;

    float max_entropy = -1.0f;
    int max_index = 0;

    std::vector<float> entropies;
    std::vector<FrameBuffer> frame_buffers;

    if ( this->isEntropyStep() )
    {
        // Entropy evaluation
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            // Draw and readback framebuffer
            auto frame_buffer = BaseClass::readback( location );

            // Output framebuffer to image file at the root node
            kvs::Timer timer( kvs::Timer::Start );
            if ( BaseClass::world().isRoot() )
            {
                const auto entropy = Controller::entropy( frame_buffer );
                entropies.push_back( entropy );
                frame_buffers.push_back( frame_buffer );

                if ( entropy > max_entropy &&
                     std::abs( entropy - max_entropy ) > 1.e-3 )
                {
                    max_entropy = entropy;
                    max_index = location.index;
                }

                //BaseClass::outputColorImage( location, frame_buffer );
            }
            timer.stop();
            entr_time += BaseClass::saveTimer().time( timer );
        }

        // Distribute the index indicates the max entropy image
        BaseClass::world().broadcast( max_index );
        BaseClass::world().broadcast( max_entropy );
        const auto& max_location = BaseClass::viewpoint().at( max_index );
        const auto max_position = max_location.position;
        const auto max_rotation = max_location.rotation;
        Controller::setMaxIndex( max_index );
        Controller::setMaxPosition( max_position );
        Controller::setMaxRotation( max_rotation );
        Controller::setMaxEntropy( max_entropy );

        // Calculate camera focus point.
        kvs::Vec3 at = BaseClass::viewpoint().at( max_index ).look_at;
        if ( BaseClass::world().isRoot() )
        {
            const auto& frame_buffer = frame_buffers[ max_index ];
            const auto at_w = this->look_at_in_window( frame_buffer );
            at = this->window_to_object( at_w, max_location );
        }

        // Readback frame buffer rendererd from updated location.
        BaseClass::world().broadcast( at.data(), sizeof(float) * 3 );
        const auto location = this->update_location( max_location, at );
        const auto frame_buffer = BaseClass::readback( location );

        // Output the rendering images and the heatmap of entropies.
        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().isRoot() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                BaseClass::outputColorImage( location, frame_buffer );
                //BaseClass::outputDepthImage( location, frame_buffer );
                BaseClass::outputEntropyTable( entropies );
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    else
    {
        auto radius = Controller::erpRadius();
        auto rotation = Controller::erpRotation();
        const size_t i = 999999;
        const auto d = InSituVis::Viewpoint::Direction::Uni;
        const auto p = kvs::Quat::Rotate( kvs::Vec3( { 0.0f, radius, 0.0f } ), rotation );
        const auto u = kvs::Quat::Rotate( kvs::Vec3( { 0.0f, 0.0f, -1.0f } ), rotation );
        const auto l = kvs::Vec3( { 0.0f, 0.0f, 0.0f } );
        const auto location = InSituVis::Viewpoint::Location( i, d, p, u, rotation, l );
        auto frame_buffer = BaseClass::readback( location );
        const auto path_entropy = Controller::entropy( frame_buffer );
        Controller::setMaxEntropy( path_entropy );
        Controller::setMaxPosition( p );

        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().rank() == BaseClass::world().root() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                BaseClass::outputColorImage( location, frame_buffer );
                //BaseClass::outputDepthImage( location, frame_buffer );
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    BaseClass::entrTimer().stamp( entr_time );
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( BaseClass::rendTime() );
    BaseClass::compTimer().stamp( BaseClass::compTime() );
}

inline kvs::Vec3 CameraFocusControlledAdaptor::look_at_in_window( const FrameBuffer& frame_buffer )
{
    const auto w = BaseClass::imageWidth(); // frame buffer width
    const auto h = BaseClass::imageHeight(); // frame buffer height
    const auto cw = w / m_ndivs.x(); // cropped frame buffer width
    const auto ch = h / m_ndivs.y(); // cropped frame buffer height

    auto get_center = [&] ( int i, int j ) -> kvs::Vec2i
    {
        return {
            static_cast<int>( i * cw + cw * 0.5 ),
            static_cast<int>( j * ch + ch * 0.5 ) };
    };

    auto get_depth = [&] ( const FrameBuffer& buffer ) -> float
    {
        const auto cx = static_cast<int>( cw * 0.5 );
        const auto cy = static_cast<int>( ch * 0.5 );
        const auto index = cx + cy * cw;

        const auto& depth_buffer = buffer.depth_buffer;
        if ( depth_buffer[ index ] < 1.0f )
        {
            return depth_buffer[ index ];
        }
        else
        {
            int counter = 0;
            float depth = 0.0f;
            for ( const auto d : depth_buffer )
            {
                if ( d < 1.0f ) { depth += d; counter++; }
            }
            return counter > 0 ? depth / counter : 1.0f;
        }
    };

    FrameBuffer cropped_buffer;
    cropped_buffer.color_buffer.allocate( cw * ch * 4 );
    cropped_buffer.depth_buffer.allocate( cw * ch );

    float max_entropy = -1.0f;
    kvs::Vec2i center{ 0, 0 };
    kvs::Real32 depth{ 0.0f };
    for ( int j = 0; j < m_ndivs.y(); j++ )
    {
        for ( int i = 0; i < m_ndivs.x(); i++ )
        {
            this->crop_frame_buffer( frame_buffer, { i, j }, &cropped_buffer );
            const auto e = Controller::entropy( cropped_buffer );
            if ( e > max_entropy )
            {
                max_entropy = e;
                center = get_center( i, j );
                depth = get_depth( cropped_buffer );
            }
        }
    }

    return { static_cast<float>( center.x() ), static_cast<float>( center.y() ), depth };
}

inline kvs::Vec3 CameraFocusControlledAdaptor::window_to_object(
    const kvs::Vec3 win,
    const Location& location )
{
    auto* manager = this->screen().scene()->objectManager();
    auto* camera = screen().scene()->camera();

    // Backup camera info.
    const auto p0 = camera->position();
    const auto a0 = camera->lookAt();
    const auto u0 = camera->upVector();
    {
        const auto p = location.position;
        const auto a = location.look_at;
        const auto u = location.up_vector;
        camera->setPosition( p, a, u );
    }

    const auto xv = kvs::Xform( camera->viewingMatrix() );
    const auto xp = kvs::Xform( camera->projectionMatrix() );
    const auto xo = manager->xform();
    const auto xm = xv * xo;

    // Restore camera info.
    camera->setPosition( p0, a0, u0 );

    auto x_to_a = [] ( const kvs::Xform& x, double a[16] )
    {
        const auto m = x.toMatrix();
        a[0] = m[0][0]; a[4] = m[0][1]; a[8]  = m[0][2]; a[12] = m[0][3];
        a[1] = m[1][0]; a[5] = m[1][1]; a[9]  = m[1][2]; a[13] = m[1][3];
        a[2] = m[2][0]; a[6] = m[2][1]; a[10] = m[2][2]; a[14] = m[2][3];
        a[3] = m[3][0]; a[7] = m[3][1]; a[11] = m[3][2]; a[15] = m[3][3];
    };

    double m[16]; x_to_a( xm, m ); // model-view matrix
    double p[16]; x_to_a( xp, p ); // projection matrix
    int v[4]; kvs::OpenGL::GetViewport( v ); // viewport

    kvs::Vec3d obj( 0.0, 0.0, 0.0 );
    kvs::OpenGL::UnProject(
        win.x(), win.y(), win.z(), m, p, v,
        &obj[0], &obj[1], &obj[2] );

    return kvs::Vec3( obj );
}

inline CameraFocusControlledAdaptor::Location
CameraFocusControlledAdaptor::update_location( const Location& location, const kvs::Vec3 at )
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

    return l;
}

inline void CameraFocusControlledAdaptor::crop_frame_buffer(
    const FrameBuffer& frame_buffer,
    const kvs::Vec2i& indices,
    FrameBuffer* cropped_frame_buffer )
{
    KVS_ASSERT( indices[0] < static_cast<int>( m_ndivs[0] ) );
    KVS_ASSERT( indices[1] < static_cast<int>( m_ndivs[1] ) );

    const auto w = BaseClass::imageWidth(); // frame buffer width
    const auto h = BaseClass::imageHeight(); // frame buffer height
    const auto cw = w / m_ndivs.x(); // cropped frame buffer width
    const auto ch = h / m_ndivs.y(); // cropped frame buffer height
    const auto ow = cw * indices[0]; // offset width for frame buffer
    const auto oh = ch * indices[1]; // offset height for frame buffer

    // Initialize frame buffer.
    cropped_frame_buffer->color_buffer.fill(0);
    cropped_frame_buffer->depth_buffer.fill(0);

    auto* dst_color_buffer = cropped_frame_buffer->color_buffer.data();
    auto* dst_depth_buffer = cropped_frame_buffer->depth_buffer.data();
    const auto offset = ow + oh * w;
    const auto* src_color_buffer = frame_buffer.color_buffer.data() + offset * 4;
    const auto* src_depth_buffer = frame_buffer.depth_buffer.data() + offset;
    for ( size_t j = 0; j < ch; j++ )
    {
        std::memcpy( dst_color_buffer, src_color_buffer, cw * 4 * sizeof( kvs::UInt8 ) );
        dst_color_buffer += cw * 4;
        src_color_buffer += w * 4;

        std::memcpy( dst_depth_buffer, src_depth_buffer, cw * sizeof( kvs::Real32 ) );
        dst_depth_buffer += cw;
        src_depth_buffer += w;
    }
}

} // end of namespace mpi

} // end of namespace local
