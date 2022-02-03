namespace local
{

namespace mpi
{

inline void Adaptor::setOutputSubImageEnabled(
    const bool enable,
    const bool enable_depth,
    const bool enable_alpha )
{
    m_enable_output_subimage = enable;
    m_enable_output_subimage_depth = enable_depth;
    m_enable_output_subimage_alpha = enable_alpha;
}

inline bool Adaptor::initialize()
{
    if ( !BaseClass::outputDirectory().create( m_world ) )
    {
        this->log() << "ERROR: " << "Cannot create output directories." << std::endl;
        return false;
    }

    const bool depth_testing = true;
    const auto width = BaseClass::imageWidth();
    const auto height = BaseClass::imageHeight();
    if ( !m_image_compositor.initialize( width, height, depth_testing ) )
    {
        this->log() << "ERROR: " << "Cannot initialize image compositor." << std::endl;
        return false;
    }

    BaseClass::screen().setSize( width, height );
    BaseClass::screen().create();

    return true;
}

inline bool Adaptor::finalize()
{
    if ( m_image_compositor.destroy() )
    {
        return BaseClass::finalize();
    }
    return false;
}

inline void Adaptor::exec( const BaseClass::SimTime sim_time )
{
    if ( this->isAnalysisStep() )
    {
        // Stack current time step.
        const auto step = static_cast<float>( BaseClass::timeStep() );
        BaseClass::tstepList().stamp( step );

        this->execPipeline();
        this->execRendering();
    }

    BaseClass::incrementTimeStep();
    BaseClass::clearObjects();
}

inline bool Adaptor::dump()
{
    auto& tstep_list = BaseClass::tstepList();
    auto& pipe_timer = BaseClass::pipeTimer();
    auto& rend_timer = BaseClass::rendTimer();
    auto& save_timer = BaseClass::saveTimer();
    auto& comp_timer = m_comp_timer;
    if ( tstep_list.title().empty() ) { tstep_list.setTitle( "Time step" ); }
    if ( pipe_timer.title().empty() ) { pipe_timer.setTitle( "Pipe time" ); }
    if ( rend_timer.title().empty() ) { rend_timer.setTitle( "Rend time" ); }
    if ( save_timer.title().empty() ) { save_timer.setTitle( "Save time" ); }
    if ( comp_timer.title().empty() ) { comp_timer.setTitle( "Comp time" ); }

    const std::string rank = kvs::String::From( this->world().rank(), 4, '0' );
    const std::string subdir = BaseClass::outputDirectory().name() + "/";
    kvs::StampTimerList timer_list;
    timer_list.push( tstep_list );
    timer_list.push( pipe_timer );
    timer_list.push( rend_timer );
    timer_list.push( save_timer );
    timer_list.push( comp_timer );
    if ( !timer_list.write( subdir + "vis_proc_time_" + rank + ".csv" ) ) return false;

    using Time = kvs::mpi::StampTimer;
    Time pipe_time_min( this->world(), pipe_timer ); pipe_time_min.reduceMin();
    Time pipe_time_max( this->world(), pipe_timer ); pipe_time_max.reduceMax();
    Time pipe_time_ave( this->world(), pipe_timer ); pipe_time_ave.reduceAve();
    Time rend_time_min( this->world(), rend_timer ); rend_time_min.reduceMin();
    Time rend_time_max( this->world(), rend_timer ); rend_time_max.reduceMax();
    Time rend_time_ave( this->world(), rend_timer ); rend_time_ave.reduceAve();
    Time save_time_min( this->world(), save_timer ); save_time_min.reduceMin();
    Time save_time_max( this->world(), save_timer ); save_time_max.reduceMax();
    Time save_time_ave( this->world(), save_timer ); save_time_ave.reduceAve();
    Time comp_time_min( this->world(), comp_timer ); comp_time_min.reduceMin();
    Time comp_time_max( this->world(), comp_timer ); comp_time_max.reduceMax();
    Time comp_time_ave( this->world(), comp_timer ); comp_time_ave.reduceAve();

    if ( !this->world().isRoot() ) return true;

    pipe_time_min.setTitle( pipe_timer.title() + " (min)" );
    pipe_time_max.setTitle( pipe_timer.title() + " (max)" );
    pipe_time_ave.setTitle( pipe_timer.title() + " (ave)" );
    rend_time_min.setTitle( rend_timer.title() + " (min)" );
    rend_time_max.setTitle( rend_timer.title() + " (max)" );
    rend_time_ave.setTitle( rend_timer.title() + " (ave)" );
    save_time_min.setTitle( save_timer.title() + " (min)" );
    save_time_max.setTitle( save_timer.title() + " (max)" );
    save_time_ave.setTitle( save_timer.title() + " (ave)" );
    comp_time_min.setTitle( comp_timer.title() + " (min)" );
    comp_time_max.setTitle( comp_timer.title() + " (max)" );
    comp_time_ave.setTitle( comp_timer.title() + " (ave)" );

    timer_list.clear();
    timer_list.push( tstep_list );
    timer_list.push( pipe_time_min );
    timer_list.push( pipe_time_max );
    timer_list.push( pipe_time_ave );
    timer_list.push( rend_time_min );
    timer_list.push( rend_time_max );
    timer_list.push( rend_time_ave );
    timer_list.push( save_time_min );
    timer_list.push( save_time_max );
    timer_list.push( save_time_ave );
    timer_list.push( comp_time_min );
    timer_list.push( comp_time_max );
    timer_list.push( comp_time_ave );

    const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
    return timer_list.write( basedir + "vis_proc_time.csv" );
}

inline void Adaptor::execRendering()
{
    m_rend_time = 0.0f;
    m_comp_time = 0.0f;
    float save_time = 0.0f;
    {
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            // Draw and readback framebuffer
            auto frame_buffer = this->readback( location );

            // Output framebuffer to image file at the root node
            kvs::Timer timer( kvs::Timer::Start );
            if ( m_world.rank() == m_world.root() )
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    const auto size = BaseClass::outputImageSize( location );
                    const auto width = size.x();
                    const auto height = size.y();
                    const auto buffer = frame_buffer.color_buffer;
                    kvs::ColorImage image( width, height, buffer );
                    image.write( this->outputFinalImageName( location ) );
                }
            }
            timer.stop();
            save_time += BaseClass::saveTimer().time( timer );
        }
    }
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( m_rend_time );
    m_comp_timer.stamp( m_comp_time );
}

inline Adaptor::FrameBuffer Adaptor::drawScreen( std::function<void(const FrameBuffer&)> func )
{
    // Draw and read-back image
    kvs::Timer timer_rend( kvs::Timer::Start );
    BaseClass::screen().draw();
    timer_rend.stop();
    m_rend_time += BaseClass::rendTimer().time( timer_rend );

    // Apply the func for partial rendering buffers before image composition.
    auto color_buffer = BaseClass::screen().readbackColorBuffer();
    auto depth_buffer = BaseClass::screen().readbackDepthBuffer();
    func( { color_buffer, depth_buffer } );

    // Image composition
    kvs::Timer timer_comp( kvs::Timer::Start );
    if ( !m_image_compositor.run( color_buffer, depth_buffer ) )
    {
        this->log() << "ERROR: " << "Cannot compose images." << std::endl;
    }
    timer_comp.stop();
    m_comp_time += m_comp_timer.time( timer_comp );

    return { color_buffer, depth_buffer };
}

inline std::string Adaptor::outputFinalImageName( const InSituVis::Viewpoint::Location& location )
{
    const auto time = BaseClass::timeStep();
    const auto space = location.index;
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_space = kvs::String::From( space, 6, '0' );

    const auto output_basename = BaseClass::outputFilename();
    const auto output_filename = output_basename + "_" + output_time + "_" + output_space;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
    return filename;
}

inline void Adaptor::outputSubImages(
    const FrameBuffer& frame_buffer,
    const InSituVis::Viewpoint::Location& location,
    const std::string& suffix )
{
    if ( m_enable_output_subimage )
    {
        const auto& color_buffer = frame_buffer.color_buffer;
        const auto& depth_buffer = frame_buffer.depth_buffer;

        // Color image
        const auto width = BaseClass::imageWidth();
        const auto height = BaseClass::imageHeight();
        kvs::ColorImage image( width, height, color_buffer );
        image.write( BaseClass::outputImageName( location, "_color_" + suffix ) );

        // Depth image
        if ( m_enable_output_subimage_depth )
        {
            kvs::GrayImage depth_image( width, height, depth_buffer );
            depth_image.write( BaseClass::outputImageName( location, "_depth_" + suffix ) );
        }

        // Alpha image
        if ( m_enable_output_subimage_alpha )
        {
            kvs::GrayImage alpha_image( width, height, color_buffer, 3 );
            alpha_image.write( BaseClass::outputImageName( location, "_alpha_" + suffix ) );
        }
    }
}

inline Adaptor::DepthBuffer Adaptor::backgroundDepthBuffer()
{
    const auto width = BaseClass::screen().width();
    const auto height = BaseClass::screen().height();
    DepthBuffer buffer( width * height );
    buffer.fill(0);
    return buffer;
}

inline Adaptor::FrameBuffer Adaptor::readback( const InSituVis::Viewpoint::Location& location )
{
    switch ( location.direction )
    {
    case InSituVis::Viewpoint::Direction::Uni: return this->readback_uni_buffer( location );
    case InSituVis::Viewpoint::Direction::Omni: return this->readback_omn_buffer( location );
    case InSituVis::Viewpoint::Direction::Adaptive: return this->readback_adp_buffer( location );
    default: return { BaseClass::backgroundColorBuffer(), this->backgroundDepthBuffer() };
    }
}

inline Adaptor::FrameBuffer Adaptor::readback_uni_buffer( const InSituVis::Viewpoint::Location& location )
{
    const auto p = location.position;
    const auto a = location.look_at;
    const auto u = location.up_vector;
    if ( p == a )
    {
        kvs::Timer timer_rend( kvs::Timer::Start );
        auto color_buffer = BaseClass::backgroundColorBuffer();
        auto depth_buffer = this->backgroundDepthBuffer();
        timer_rend.stop();
        m_rend_time += BaseClass::rendTimer().time( timer_rend );
        return { color_buffer, depth_buffer };
    }
    else
    {
        auto* camera = BaseClass::screen().scene()->camera();
        auto* light = BaseClass::screen().scene()->light();

        // Backup camera and light info.
        const auto p0 = camera->position();
        const auto a0 = camera->lookAt();
        const auto u0 = camera->upVector();

        //Draw the scene.
        camera->setPosition( p, a, u );
        light->setPosition( p );
        const auto buffer = this->drawScreen();


        // Restore camera and light info.
        camera->setPosition( p0, a0, u0 );
        light->setPosition( p0 );

        return buffer;
    }
}

inline Adaptor::FrameBuffer Adaptor::readback_omn_buffer( const InSituVis::Viewpoint::Location& location )
{
    using SphericalColorBuffer = InSituVis::SphericalBuffer<kvs::UInt8>;
    using SphericalDepthBuffer = InSituVis::SphericalBuffer<kvs::Real32>;

    auto* camera = BaseClass::screen().scene()->camera();
    auto* light = BaseClass::screen().scene()->light();

    // Backup camera and light info.
    const auto fov = camera->fieldOfView();
    const auto front = camera->front();
    const auto cp = camera->position();
    const auto ca = camera->lookAt();
    const auto cu = camera->upVector();
    const auto lp = light->position();
    const auto& p = location.position;

    // Draw the scene.
    camera->setFieldOfView( 90.0 );
    camera->setFront( 0.1 );
    light->setPosition( p );

    float rend_time = 0.0f;
    float comp_time = 0.0f;
    SphericalColorBuffer color_buffer( BaseClass::screen().width(), BaseClass::screen().height() );
    SphericalDepthBuffer depth_buffer( BaseClass::screen().width(), BaseClass::screen().height() );
    for ( size_t i = 0; i < SphericalColorBuffer::Direction::NumberOfDirections; i++ )
    {
        // Rendering.
        const auto d = SphericalColorBuffer::Direction(i);
        const auto dir = SphericalColorBuffer::DirectionVector(d);
        const auto up = SphericalColorBuffer::UpVector(d);
        camera->setPosition( p, p + dir, up );
        const auto buffer = this->drawScreen(
            [&] ( const FrameBuffer& frame_buffer )
            {
                // Output rendering image (partial rendering image) for each direction
                const auto dname = SphericalColorBuffer::DirectionName(d);
                this->outputSubImages( frame_buffer, location, dname );
            } );

        color_buffer.setBuffer( d, buffer.color_buffer );
        depth_buffer.setBuffer( d, buffer.depth_buffer );
    }

    m_rend_time = rend_time;
    m_comp_time = comp_time;

    // Restore camera and light info.
    camera->setFieldOfView( fov );
    camera->setFront( front );
    camera->setPosition( cp, ca, cu );
    light->setPosition( lp );

    // Return frame buffer
    return { color_buffer.stitch<4>(), depth_buffer.stitch<1>() };
}

inline Adaptor::FrameBuffer Adaptor::readback_adp_buffer( const InSituVis::Viewpoint::Location& location )
{
    const auto* object = BaseClass::screen().scene()->objectManager();
    return BaseClass::isInsideObject( location.position, object ) ?
        this->readback_omn_buffer( location ) :
        this->readback_uni_buffer( location );
}

} // end of namespace mpi

} // end of namespace InSituVis
