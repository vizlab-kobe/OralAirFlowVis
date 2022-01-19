#include <kvs/Math>
#include <kvs/Stat>
#include <kvs/Quaternion>

namespace
{

inline kvs::Vec3 calcUpVector( const kvs::Vec3& xyz )
{
    const float x = xyz[0];
    const float y = xyz[1];
    const float z = xyz[2];
    kvs::Vec3 u;

    if( ( x == 0 ) && ( z == 0 ) ){
        u = kvs::Vec3( { 0.0f, 0.0f, -1.0f } );
    }
    else
    {
        float r, t, p;
        r = sqrt( xyz.dot( xyz ) );
        t = acos( y / r );
        if ( x >= 0 )
        {
            p = acos( z / sqrt( z * z + x * x ) );
        }
        else
        {
            p = -1 * acos( z / sqrt( z * z + x * x ) );
        }
        if ( p < 0 )
        {
            p += 2 * kvs::Math::pi;
        }
                        
        auto rtp = kvs::Vec3( { r, t, p } );
                        
        kvs::Vec3 pp;
        if ( rtp[1] > kvs::Math::pi / 2 )
        {
            pp = rtp - kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
        }
        else
        {
            pp = rtp + kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
        }
        const float pp_x = pp[0] * std::sin( pp[1] ) * std::sin( pp[2] );
        const float pp_y = pp[0] * std::cos( pp[1] );
        const float pp_z = pp[0] * std::sin( pp[1] ) * std::cos( pp[2] );
        if ( rtp[1] > kvs::Math::pi / 2 )
        {
            u = kvs::Vec3( { pp_x, pp_y, pp_z } );
        }
        else
        {
            u = -1 * kvs::Vec3( { pp_x, pp_y, pp_z } );
        }
    }

    return u;
}

}

namespace local
{

inline float EntropyTimestepController::Entropy( const FrameBuffer& frame_buffer )
{
    const float p = 0.5f;
    return p * ColorEntropy( frame_buffer ) + ( 1 - p ) * DepthEntropy( frame_buffer );
}

inline float EntropyTimestepController::ColorEntropy( const FrameBuffer& frame_buffer )
{
    kvs::ValueArray<size_t> histogram_R( 256 );
    kvs::ValueArray<size_t> histogram_G( 256 );
    kvs::ValueArray<size_t> histogram_B( 256 );
    histogram_R.fill( 0 );
    histogram_G.fill( 0 );
    histogram_B.fill( 0 );

    size_t n = 0;
    const auto& depth_buffer = frame_buffer.depth_buffer;
    const auto& color_buffer = frame_buffer.color_buffer;
    const auto length = depth_buffer.size();
    for ( size_t i = 0; i < length; i++ )
    {
        const auto depth = depth_buffer[i];
        const auto color_R = color_buffer[ 3 * i ];
        const auto color_G = color_buffer[ 3 * i + 1 ];
        const auto color_B = color_buffer[ 3 * i + 2 ];

        if ( depth < 1.0f )
        {
            histogram_R[ color_R ] += 1;
            histogram_G[ color_G ] += 1;
            histogram_B[ color_B ] += 1;
            n += 1;
        }
    }

    float entropy_R = 0.0f;
    float entropy_G = 0.0f;
    float entropy_B = 0.0f;
    const auto log2 = std::log( 2.0f );
    for ( const auto h : histogram_R )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy_R -= p * std::log( p ) / log2; }
    }
    for ( const auto h : histogram_G )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy_G -= p * std::log( p ) / log2; }
    }
    for ( const auto h : histogram_B )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy_B -= p * std::log( p ) / log2; }
    }

    const auto entropy = ( entropy_R + entropy_G + entropy_B ) / 3.0f;

    return entropy;
}

inline float EntropyTimestepController::DepthEntropy( const FrameBuffer& frame_buffer )
{
    /*
    for ( size_t i = 0; i < ( width * height ); i++ )
    {
        if ( depth_buffer[i] < 1.0f )
        {
            const size_t j = depth_buffer[i] * 256;
            histogram[j] += 1;
            n += 1;
        }
    }

    float entropy = 0.0;
    for ( size_t i = 0; i < 256; i++ )
    {
        const float p = static_cast<float>( histogram[i] ) / n;
        if( p > 0 )
        {
            entropy -= p * log( p ) / log( 2.0f );
        }
    }
    */

    kvs::ValueArray<size_t> histogram( 256 );
    histogram.fill( 0 );

    size_t n = 0;
    const auto& depth_buffer = frame_buffer.depth_buffer;
    for ( const auto depth : depth_buffer )
    {
        if ( depth < 1.0f )
        {
            const auto i = static_cast<int>( depth * 256 );
            histogram[i] += 1;
            n += 1;
        }
    }

    float entropy = 0.0f;
    const auto log2 = std::log( 2.0f );
    for ( const auto h : histogram )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy -= p * std::log( p ) / log2; }
    }

    return entropy;
}

inline InSituVis::Viewpoint EntropyTimestepController::CreatePath(
    const kvs::Vec3& position_prv,
    const kvs::Vec3& upVector_prv,
    const kvs::Vec3& position_crr,
    const kvs::Vec3& upVector_crr,
    const size_t point_interval )
{
    using Viewpoint = InSituVis::Viewpoint;
    const kvs::Vec3 l = { 0.0f, 0.0f, 0.0f };
    const auto dir = Viewpoint::Direction::Uni;
    auto path = Viewpoint();
    //kvs::ValueArray<size_t> num_point( point_interval - 1 );
    //num_point.fill( 0 );

    {
        if ( position_prv == position_crr )
        {
            for ( size_t i = 0; i < point_interval - 1; i++ )
            {
                path.add( { dir, position_prv, upVector_prv, l } );
                //num_point[i] += 1;
            }
        }
        else
        {
            /*
            const auto q_rot = kvs::Quaternion::RotationQuaternion( position_prv, position_crr );
            const auto axis = q_rot.axis();
            const auto angle = q_rot.angle();
            const auto da = angle / ( point_interval - 1 );

            if( ( axis[1] == 0 ) && ( ( position_prv[0] * position_crr[0] < 0 ) || ( position_prv[2] * position_crr[2] < 0 ) ) )
            {
                auto R = sqrt( position_prv.dot( position_prv ) );
                if( position_crr[1] < -1.0f * position_prv[1] ) { R = -1.0f * R; }
                const auto pole = kvs::Vec3( { 0, R, 0 } );

                const auto r_prv = sqrt( position_prv[0] * position_prv[0] + position_prv[2] * position_prv[2] );
                const auto r_crr = sqrt( position_crr[0] * position_crr[0] + position_crr[2] * position_crr[2] );
                size_t n_prv = point_interval * r_prv / ( r_prv + r_crr );
                size_t n_crr = point_interval * r_crr / ( r_prv + r_crr );
                if( n_prv + n_crr == point_interval )
                {
                    if( n_prv > n_crr ) { n_prv -= 1; }
                    else { n_crr -= 1; }
                }

                const auto q_rot_prv_pole = kvs::Quaternion::RotationQuaternion( position_prv, pole );
                const auto axis_prv_pole = q_rot_prv_pole.axis();
                const auto angle_prv_pole = q_rot_prv_pole.angle();
                const auto da_prv_pole = angle_prv_pole / n_prv;

                for( size_t i = 1; i <= n_prv; i++ )
                {
                    const auto xyz = kvs::Quaternion::Rotate( position_prv, axis_prv_pole, da_prv_pole * i );
                    const size_t middle_point = abs( xyz[1] );
                    const auto dda = da_prv_pole / ( middle_point + 1 );

                    if( middle_point > 0 ){
                        for( size_t j = 1; j <= middle_point; j++ )
                        {
                            const auto m_xyz = kvs::Quaternion::Rotate( position_prv, axis_prv_pole, da_prv_pole * ( i - 1 ) + dda * j );
                            const auto m_u = calcUpVector( m_xyz );
                            path.add( { dir, m_xyz, m_u, l } );
                            num_point[ i - 1 ] += 1;
                        }
                    }

                    kvs::Vec3 u;
                    if( i == n_prv ) { u = axis; }
                    else { u = calcUpVector( xyz ); }

                    path.add( { dir, xyz, u, l } );
                    num_point[ i - 1 ] += 1;
                }

                const auto q_rot_pole_crr = kvs::Quaternion::RotationQuaternion( pole, position_crr );
                const auto axis_pole_crr = q_rot_pole_crr.axis();
                const auto angle_pole_crr = q_rot_pole_crr.angle();
                const auto da_pole_crr = angle_pole_crr / n_crr;

                for( size_t i = 1; i <= n_crr; i++ )
                {
                    const auto xyz = kvs::Quaternion::Rotate( pole, axis_pole_crr, da_pole_crr * i );
                    const size_t middle_point = abs( xyz[1] );
                    const auto dda = da_pole_crr / ( middle_point + 1 );

                    if( middle_point > 0 ){
                        for( size_t j = 1; j <= middle_point; j++ )
                        {
                            const auto m_xyz = kvs::Quaternion::Rotate( pole, axis_pole_crr, da_pole_crr * ( i - 1 ) + dda * j );
                            const auto m_u = calcUpVector( m_xyz );
                            path.add( { dir, m_xyz, m_u, l } );
                            num_point[ n_prv + i - 1 ] += 1;
                        }
                    }

                    if( i < n_crr ){
                        const auto u = calcUpVector( xyz );
                        path.add( { dir, xyz, u, l } );
                        num_point[ n_prv + i - 1 ] += 1;
                    }
                }
            }
            else
            {
                for ( size_t i = 1; i < point_interval; i++ )
                {
                    const auto xyz = kvs::Quaternion::Rotate( position_prv, axis, da * i );
                    const size_t middle_point = abs( xyz[1] );
                    const auto dda = da / ( middle_point + 1 );

                    if( middle_point > 0 ){
                        for( size_t j = 1; j <= middle_point; j++ )
                        {
                            const auto m_xyz = kvs::Quaternion::Rotate( position_prv, axis, da * ( i - 1 ) + dda * j );
                            const auto m_u = calcUpVector( m_xyz );
                            path.add( { dir, m_xyz, m_u, l } );
                            num_point[ i - 1 ] += 1;
                        }
                    }

                    if( i < point_interval ){
                        const auto u = calcUpVector( xyz );
                        path.add( { dir, xyz, u, l } );
                        num_point[ i - 1 ] += 1;
                    }
                }
            }*/

            const auto q_rot = kvs::Quaternion::RotationQuaternion( position_prv, position_crr );
            const auto axis = q_rot.axis();
            const auto angle = q_rot.angle();
            const auto da = angle / point_interval;

            for ( size_t i = 1; i < point_interval; i++ )
            {
                const auto xyz = kvs::Quaternion::Rotate( position_prv, axis, da * i );
                const auto u = calcUpVector( xyz );

                //const auto pp = kvs::Quaternion::Rotate( pp_prv, pp_axis, pp_da * i );
                //const auto u = pp - xyz;
                path.add( { dir, xyz, u, l } );
            }
        }
    }

    //m_num_point = num_point;
    return path;
}

inline void EntropyTimestepController::push( const Data& data )
{
    if ( m_data_queue.empty() && m_previous_data.empty() )
    {
        // Initial step.
        m_previous_position = this->process( data );
        m_previous_data = data;
        this->setPrvUpVector( this->crrUpVector() );
    }
    else
    {
        if ( this->isCacheEnabled() )
        {
            if ( m_data_queue.size() < ( m_interval - 1 ) )
            {
                m_data_queue.push( data );
            }
            else
            {
                m_current_position = this->process( data );

                m_path = this->CreatePath(
                    m_previous_position,
                    this->prvUpVector(),
                    m_current_position,
                    this->crrUpVector(),
                    m_interval );

                const auto length = m_data_queue.size();
                for ( size_t i = 0; i < length; i++ )
                {
                    const auto data_front = m_data_queue.front();
                    this->process( data_front, m_path, i );
                    m_data_queue.pop();
                }

                m_previous_position = m_current_position;
                this->setPrvUpVector( crrUpVector() );
                DataQueue().swap( m_data_queue );
                m_path.clear();
            }
        }
    }
}

float EntropyTimestepController::entropy( const FrameBuffer& frame_buffer )
{
    return m_entropy_function( frame_buffer );
}

} // end of namespace InSituVis
