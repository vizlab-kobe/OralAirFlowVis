#include <kvs/ColorImage>
#include <kvs/RGBColor>


namespace local
{

namespace mpi
{

inline void CameraFocusControlledAdaptor::setOutputEvaluationImageEnabled(
    const bool enable,
    const bool enable_depth )
{
    m_enable_output_evaluation_image = enable;
    m_enable_output_evaluation_image_depth = enable_depth;
}

inline bool CameraFocusControlledAdaptor::isEntropyStep()
{
    return BaseClass::timeStep() % ( BaseClass::analysisInterval() * Controller::entropyInterval() ) == 0;
}

inline bool CameraFocusControlledAdaptor::isFinalTimeStep()
{
    return BaseClass::timeStep() == m_final_time_step;
}

inline bool CameraFocusControlledAdaptor::dump()
{
    bool ret = true;
    bool ret_f = true;
    if ( BaseClass::world().isRoot() )
    {
        if ( m_entr_timer.title().empty() ) { m_entr_timer.setTitle( "Ent time" ); }
        kvs::StampTimerList timer_list;
        timer_list.push( m_entr_timer );

        if ( m_focus_timer.title().empty() ) { m_focus_timer.setTitle( "focus time" ); }//gamennbukatu
        kvs::StampTimerList f_timer_list;
        f_timer_list.push( m_focus_timer );

        const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
        ret = timer_list.write( basedir + "ent_proc_time.csv" );
        ret_f = f_timer_list.write( basedir + "focus_proc_time.csv" );

        this->outputPathEntropies( Controller::pathEntropies() );
        this->outputPathPositions( Controller::pathPositions() );
    }

    return BaseClass::dump() && ret && ret_f;
}

inline void CameraFocusControlledAdaptor::exec( const BaseClass::SimTime sim_time )
{
    Controller::setCacheEnabled( BaseClass::isAnalysisStep() );
    Controller::push( BaseClass::objects() );

    BaseClass::incrementTimeStep();
    if( this->isFinalTimeStep())
    {
        Controller::setFinalStep( true );
        const auto dummy = Data();
        Controller::push( dummy );
    }
    BaseClass::clearObjects();
}

inline void CameraFocusControlledAdaptor::execRendering()
{
    BaseClass::setRendTime( 0.0f );
    BaseClass::setCompTime( 0.0f );
    float save_time = 0.0f;
    float entr_time = 0.0f;
    float focus_time = 0.0f;

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

                /*if ( entropy > max_entropy )
                {
                    max_entropy = entropy;
                    max_index = location.index;
                }*/

                //add
                if ( entropy > max_entropy &&
                     std::abs( entropy - max_entropy ) > 1.e-3 )
                {
                    max_entropy = entropy;
                    max_index = location.index;
                }
                //


                //this->outputColorImage( location, frame_buffer );
            }
            timer.stop();
            entr_time += BaseClass::saveTimer().time( timer );
        }

        // Distribute the index indicates the max entropy image
        BaseClass::world().broadcast( max_index );
        BaseClass::world().broadcast( max_entropy );
        //const auto max_position = BaseClass::viewpoint().at( max_index ).position;
        //const auto max_rotation = BaseClass::viewpoint().at( max_index ).rotation;

        //add
        const auto& max_location = BaseClass::viewpoint().at( max_index );
        const auto max_position = max_location.position;
        const auto max_rotation = max_location.rotation;

        //
        Controller::setMaxIndex( max_index );
        Controller::setMaxRotation( max_rotation );
        Controller::setMaxEntropy( max_entropy );

        //add
        // Calculate camera focus point.
        kvs::Vec3 at = BaseClass::viewpoint().at( max_index ).look_at;
        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().isRoot() )
        {  
            const auto& frame_buffer = frame_buffers[ max_index ];
            const auto at_w = this->look_at_in_window( frame_buffer );
            at = this->window_to_object( at_w, max_location );
        }
        timer.stop();
        focus_time += BaseClass::saveTimer().time( timer );

        // Readback frame buffer rendererd from updated location.
        BaseClass::world().broadcast( at.data(), sizeof(float) * 3 );
        Controller::setMaxFocusPoint( at );
        auto location = this->update_location( max_location, at );
        Controller::setMaxPosition( max_position );
        //To get zoom function
        //const kvs::Vec3 shortest_dis = ( at - max_position ) / m_zoom_divid;
        for(int space=0; space<m_zoom_divid; space++ ){
            float t = static_cast<float>(space) / static_cast<float>(m_zoom_divid); 
            auto new_p = (1 - t) * max_position + t * at;
            location.position = new_p; 
            auto frame_buffer = BaseClass::readback( location );

            // Output the rendering images and the heatmap of entropies.
            kvs::Timer timer( kvs::Timer::Start );
            if ( BaseClass::world().isRoot() )
            {
            /*
            if ( BaseClass::isOutputImageEnabled() )
            {
                const auto index = Controller::maxIndex();
                const auto& location = BaseClass::viewpoint().at( index );
                const auto& frame_buffer = frame_buffers[ index ];
                this->outputColorImage( location, frame_buffer );
                //this->outputDepthImage( location, frame_buffer );
                this->outputEntropyTable( entropies );
            }*/
            //add
                if ( BaseClass::isOutputImageEnabled() )
                {  
                    this->outputColorImage( location, frame_buffer, space );                
                    //BaseClass::outputDepthImage( location, frame_buffer );
                    this->outputEntropyTable( entropies );
                }
            //
            }
            timer.stop();
            save_time += BaseClass::saveTimer().time( timer );
        }
    }
    else
    {
        auto radius = Controller::erpRadius();
        auto rotation = Controller::erpRotation();
        auto focus = Controller::erpFocus();//add
        const size_t i = 999999;
        const auto d = InSituVis::Viewpoint::Direction::Uni;
        const auto p = kvs::Quat::Rotate( kvs::Vec3( { 0.0f, radius, 0.0f } ), rotation );
        const auto u = kvs::Quat::Rotate( kvs::Vec3( { 0.0f, 0.0f, -1.0f } ), rotation );
        const auto l = kvs::Vec3( { 0.0f, 0.0f, 0.0f } );
        auto location = InSituVis::Viewpoint::Location( i, d, p, u, rotation, l );//add
        location = this->update_location( location, focus );//add
        //const auto location = InSituVis::Viewpoint::Location( i, d, p, u, rotation, l );
        Controller::setMaxFocusPoint( focus );//add
        Controller::setMaxPosition( p );
        for(int space=0; space<m_zoom_divid; space++ ){
            float t = static_cast<float>(space) / static_cast<float>(m_zoom_divid); 
            auto new_p = (1 - t) * p + t * focus;
            //log()<<new_p<<std::endl;
            location.position = new_p; 
            auto frame_buffer = BaseClass::readback( location );
            
            if(space == 0)
            { 
            const auto path_entropy = Controller::entropy( frame_buffer );
            Controller::setMaxEntropy( path_entropy );
            }

           kvs::Timer timer( kvs::Timer::Start );
           if ( BaseClass::world().rank() == BaseClass::world().root() )   
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    this->outputColorImage( location, frame_buffer, space );
                    //this->outputDepthImage( location, frame_buffer );
                }
            }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
        }
    }
    m_entr_timer.stamp( entr_time );
    m_focus_timer.stamp( focus_time );
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( BaseClass::rendTime() );
    BaseClass::compTimer().stamp( BaseClass::compTime() );
}

inline void CameraFocusControlledAdaptor::outputColorImage(
    const InSituVis::Viewpoint::Location& location,
    const FrameBuffer& frame_buffer,
    const int space )
{
    const auto size = BaseClass::outputImageSize( location );
    const auto buffer = frame_buffer.color_buffer;
    kvs::ColorImage image( size.x(), size.y(), buffer );
    image.write( this->outputFinalImageName( space ) );
}

inline std::string CameraFocusControlledAdaptor::outputFinalImageName( const int space )
{
    const auto time = BaseClass::timeStep();
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_basename = BaseClass::outputFilename();
    const auto output_space = kvs::String::From( space, 6, '0' );
    const auto output_filename = output_basename + "_" + output_time + "_" + output_space;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
    return filename;
}

//add
inline kvs::Vec3 CameraFocusControlledAdaptor::look_at_in_window( const FrameBuffer& frame_buffer )
{
    const auto w = BaseClass::imageWidth(); // frame buffer width
    const auto h = BaseClass::imageHeight(); // frame buffer height
    const auto cw = w / m_ndivs.x(); // cropped frame buffer width
    const auto ch = h / m_ndivs.y(); // cropped frame buffer height
    std::vector<float> focus_entropies;

    auto get_center = [&] ( int i, int j ) -> kvs::Vec2i
    {
        return {
            static_cast<int>( i * cw + cw * 0.5 ),
            static_cast<int>( h - ( j * ch + ch * 0.5 ) ) };
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
            focus_entropies.push_back(e);
            if ( e > max_entropy &&
                 std::abs( e - max_entropy ) > 1.e-3 )
            {
                max_entropy = e;
                center = get_center( i, j );
                depth = get_depth( cropped_buffer );
            }
        }
    }
    this->outputFocusEntropyTable( focus_entropies );

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
//

inline void CameraFocusControlledAdaptor::process( const Data& data )
{
    BaseClass::execPipeline( data );
    this->execRendering();
}

//inline void CameraFocusControlledAdaptor::process( const Data& data, const float radius, const kvs::Quaternion& rotation )
inline void CameraFocusControlledAdaptor::process( const Data& data, const float radius, const kvs::Quaternion& rotation , const kvs::Vec3& foc) //add
{
    const auto current_step = BaseClass::timeStep();
    {
        // Reset time step, which is used for output filename,
        // for visualizing the stacked dataset.
        const auto L_crr = Controller::dataQueue().size();
        if ( L_crr > 0 )
        {
            const auto l = BaseClass::analysisInterval();
            const auto step = current_step - L_crr * l;
            BaseClass::setTimeStep( step );
        }

        // Stack current time step.
        const auto step = static_cast<float>( BaseClass::timeStep() );
        BaseClass::tstepList().stamp( step );

        // Execute vis. pipeline and rendering.
        Controller::setErpRotation( rotation );
        Controller::setErpRadius( radius );
        Controller::setErpFocus( foc );//add
        BaseClass::execPipeline( data );
        this->execRendering();
    }
    BaseClass::setTimeStep( current_step );
}


inline void CameraFocusControlledAdaptor::outputDepthImage(
    const InSituVis::Viewpoint::Location& location,
    const FrameBuffer& frame_buffer )
{
    const auto size = BaseClass::outputImageSize( location );
    const auto buffer = frame_buffer.depth_buffer;
    kvs::GrayImage image( size.x(), size.y(), buffer );
    image.write( BaseClass::outputFinalImageName( location ) );
}

inline void CameraFocusControlledAdaptor::outputEntropyTable(
    const std::vector<float> entropies )
{
    const auto time = BaseClass::timeStep();
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_filename =  "output_entropy_table_" + output_time;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream table( filename );

    const size_t dim = BaseClass::viewpoint().numberOfLocations();
    size_t dim_long = 1;
    auto y0 = BaseClass::viewpoint().at( 0 ).position[1];
    for( size_t i = 1; i < dim; i++ )
    {
        auto y = BaseClass::viewpoint().at( i ).position[1];
        if( y != y0 )
        {
            dim_long = i;
            break;
        }
    }

    for( size_t i = 0; i < dim; i++ )
    {
        table << entropies[i];
        table << ",";
        if( ( i + 1 ) % dim_long == 0 )
        {
            table << std::endl;
        }
    }

    table.close();
}

inline void CameraFocusControlledAdaptor::outputFocusEntropyTable(
    const std::vector<float> entropies )
{
    const auto time = BaseClass::timeStep();
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_filename =  "output_focus_entropy_table_" + output_time;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream table( filename );

    const size_t width = m_ndivs.x();
    const size_t height = m_ndivs.y();

    for( size_t i = 0; i < height; i++ )
    {
        for(size_t j = 0; j < width; j++){
            table << entropies[width*i+j];
            if(j==width-1) break;
            table << ",";
        }
            table << std::endl;
    }
    table.close();
}

inline void CameraFocusControlledAdaptor::outputPathEntropies(
    const std::vector<float> path_entropies )
{
    const auto output_filename =  "output_path_entropies";
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream path_entropy( filename );
    const auto interval = BaseClass::analysisInterval();

    path_entropy << "Time,Entropy" << std::endl;
    for( size_t i = 0; i < path_entropies.size(); i++ )
    {
        path_entropy << interval * i << "," << path_entropies[i] << std::endl;
    }

    path_entropy.close();
}

inline void CameraFocusControlledAdaptor::outputPathPositions(
    const std::vector<float> path_positions )
{
    const auto output_filename =  "output_path_positions";
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".csv";
    std::ofstream position( filename );
    const auto interval = BaseClass::analysisInterval();

    position << "Time,X,Y,Z" << std::endl;
    for( size_t i = 0; i < path_positions.size() / 3; i++ )
    {
        const auto x = path_positions[ 3 * i ];
        const auto y = path_positions[ 3 * i + 1 ];
        const auto z = path_positions[ 3 * i + 2 ];
        position << interval * i << "," << x << "," << y << "," << z << std::endl;
    }

    position.close();
}

} // end of namespace mpi

} // end of namespace InSituVis
