#pragma once
#include <kvs/OrthoSlice>
#include <kvs/Isosurface>
#include <kvs/PolygonObject>
#include <kvs/PolygonRenderer>
#include <kvs/PolygonImporter>
#include <kvs/UnstructuredVolumeObject>
#include <kvs/ExternalFaces>
#include <kvs/Bounds>
#include <kvs/String>
#include <kvs/StampTimer>
#include <kvs/StampTimerList>
#include <kvs/Math>
#include <kvs/mpi/StampTimer>
#include <InSituVis/Lib/Adaptor.h>
#include <InSituVis/Lib/Viewpoint.h>
#include <InSituVis/Lib/CubicViewpoint.h>
#include <InSituVis/Lib/SphericalViewpoint.h>


//#define IN_SITU_VIS__ADAPTIVE_TIMESTEP_CONTROLL
//#define IN_SITU_VIS__STOCHASTIC_RENDERING

#if defined( IN_SITU_VIS__ADAPTIVE_TIMESTEP_CONTROLL )
#include <InSituVis/Lib/TimestepControlledAdaptor.h>
namespace { using Adaptor = InSituVis::mpi::TimestepControlledAdaptor; }
#elif defined( IN_SITU_VIS__STOCHASTIC_RENDERING )
#include <InSituVis/Lib/StochasticRenderingAdaptor.h>
#include <kvs/StochasticLineRenderer>
#include <kvs/StochasticPolygonRenderer>
#include <kvs/ParticleBasedRenderer>
#include <kvs/CellByCellMetropolisSampling>
namespace { using Adaptor = InSituVis::mpi::StochasticRenderingAdaptor; }
#else
namespace { using Adaptor = InSituVis::mpi::Adaptor; }
#endif


namespace local
{

class InSituVis : public ::Adaptor
{
    using BaseClass = ::Adaptor;
    using Object = BaseClass::Object;
    using Volume = kvs::UnstructuredVolumeObject;
    using Screen = BaseClass::Screen;

public:
    static Pipeline OrthoSlice();
    static Pipeline Isosurface();
    static Pipeline Surface( const kvs::mpi::Communicator& world );
#if defined( IN_SITU_VIS__STOCHASTIC_RENDERING )
    static Pipeline ParticleBasedRendering( const size_t repeats );
#endif

private:
    kvs::PolygonObject m_boundary_mesh; ///< boundary mesh
    kvs::mpi::StampTimer m_sim_timer{ BaseClass::world() }; ///< timer for sim. process
    kvs::mpi::StampTimer m_cnv_timer{ BaseClass::world() }; ///< timer for cnv. process
    kvs::mpi::StampTimer m_vis_timer{ BaseClass::world() }; ///< timer for vis. process

public:
    InSituVis( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root )
    {
        // Common parameters.
        //enum { Ortho, Iso, Surf } pipeline_type = Ortho; // 'Ortho', 'Iso' or 'Surf'
        enum { Ortho, Iso, Surf } pipeline_type = Surf; // 'Ortho', 'Iso' or 'Surf'
        enum { Single, Dist } viewpoint_type = Single; // 'Single' or 'Dist'
        //enum { Single, Dist } viewpoint_type = Dist; // 'Single' or 'Dist'
        this->setImageSize( 1024, 1024 );
        //this->setImageSize( 512, 512 );
        this->setOutputImageEnabled( true );
        this->setOutputSubImageEnabled( false, false, false ); // color, depth, alpha
        //this->setOutputSubImageEnabled( true, false, false ); // color, depth, alpha
        //this->setOutputSubImageEnabled( true, true, true ); // color, depth, alpha

        // Time intervals.
        this->setAnalysisInterval( 3 ); // l: analysis time interval
#if defined( IN_SITU_VIS__ADAPTIVE_TIMESTEP_CONTROLL )
        this->setValidationInterval( 4 ); // L: validation time interval
        this->setSamplingGranularity( 2 ); // R: granularity for the pattern A
        this->setDivergenceThreshold( 0.01 );
#endif

        // Set visualization pipeline.
#if defined( IN_SITU_VIS__STOCHASTIC_RENDERING )
        const size_t repeats = 100;
        this->setRepetitionLevel( repeats );
        this->setPipeline( local::InSituVis::ParticleBasedRendering( repeats ) );
#else
        switch ( pipeline_type )
        {
        case Ortho:
            this->setPipeline( local::InSituVis::OrthoSlice() );
            break;
        case Iso:
            this->setPipeline( local::InSituVis::Isosurface() );
            break;
        case Surf:
            this->setPipeline( local::InSituVis::Surface( BaseClass::world() ) );
            break;
        default: break;
        }
#endif

        // Set viewpoint(s)
        switch ( viewpoint_type )
        {
        case Single:
        {
            using Viewpoint = ::InSituVis::Viewpoint;
            //auto location = Viewpoint::Location( {0, 0, 12} );
            auto location = Viewpoint::Location( {-7, 7, 7} );
            auto vp = Viewpoint( location );
            this->setViewpoint( vp );
            break;
        }
        case Dist:
        {
            using Viewpoint = ::InSituVis::CubicViewpoint;
            auto dims = kvs::Vec3ui( 3, 3, 3 );
            auto dir = Viewpoint::Direction::Uni;
            auto vp = Viewpoint();
            vp.setDims( dims );
            vp.create( dir );
            this->setViewpoint( vp );
            break;
        }
        default: break;
        }
    }

    kvs::mpi::StampTimer& simTimer() { return m_sim_timer; }
    kvs::mpi::StampTimer& cnvTimer() { return m_cnv_timer; }
    kvs::mpi::StampTimer& visTimer() { return m_vis_timer; }

    void exec( const BaseClass::SimTime sim_time )
    {
        if ( !BaseClass::screen().scene()->hasObject( "BoundaryMesh") )
        {
            const bool visible = BaseClass::world().rank() == BaseClass::world().root ();
            auto* object = new kvs::PolygonObject();
            object->shallowCopy( m_boundary_mesh );
            object->setName( "BoundaryMesh" );
            object->setVisible( visible );

            // Register the bounding box at the root rank.
#if defined( IN_SITU_VIS__STOCHASTIC_RENDERING )
            // Bounding box
            kvs::Bounds bounds( kvs::RGBColor::Black(), 1.0f );
            auto* o = bounds.outputLineObject( object );
            o->setVisible( object->isVisible() );
            auto* r = new kvs::StochasticLineRenderer();
            BaseClass::screen().registerObject( o, r );

            // Boundary mesh
            object->setOpacity( 30 );
            auto* renderer = new kvs::StochasticPolygonRenderer();
            renderer->setTwoSideLightingEnabled( true );
            BaseClass::screen().registerObject( object, renderer );
#else
            // Bounding box
            auto* renderer = new kvs::Bounds();
            BaseClass::screen().registerObject( object, renderer );
#endif
        }

        const auto update_min_max_values = false;
        if ( update_min_max_values )
        {
            auto min_value = Volume::DownCast( *BaseClass::objects().begin() )->minValue();
            auto max_value = Volume::DownCast( *BaseClass::objects().begin() )->maxValue();
            for ( auto& object : BaseClass::objects() )
            {
                auto* volume = Volume::DownCast( object.get() );
                volume->updateMinMaxValues();

                min_value = kvs::Math::Min( min_value, volume->minValue() );
                max_value = kvs::Math::Max( max_value, volume->maxValue() );
            }

            BaseClass::world().allReduce( min_value, min_value, MPI_MIN );
            BaseClass::world().allReduce( max_value, max_value, MPI_MAX );

            for ( auto& object : BaseClass::objects() )
            {
                auto* volume = Volume::DownCast( object.get() );
                volume->setMinMaxValues( min_value, max_value );
            }
        }

        BaseClass::exec( sim_time );
    }

    void importBoundaryMesh( const std::string& filename )
    {
        m_boundary_mesh = kvs::PolygonImporter( filename );

        // Scaling coordinate values of the boundary object adjusing to
        // the coordinate scale of the volume dataset.
        const auto scale = 1.0f / 1000.0f;
        const auto min_coord = m_boundary_mesh.minObjectCoord() * scale;
        const auto max_coord = m_boundary_mesh.maxObjectCoord() * scale;
        auto coords = m_boundary_mesh.coords();
        for ( auto& p : coords ) { p *= scale; }
        m_boundary_mesh.setCoords( coords );
        m_boundary_mesh.setMinMaxObjectCoords( min_coord, max_coord );
        m_boundary_mesh.setMinMaxExternalCoords( min_coord, max_coord );
    }

    bool dump()
    {
        if ( !BaseClass::dump() ) return false;

        // For each node
        m_sim_timer.setTitle( "Sim time" );
        m_cnv_timer.setTitle( "Cnv time" );
        m_vis_timer.setTitle( "Vis time" );

        const std::string rank = kvs::String::From( this->world().rank(), 4, '0' );
        const std::string subdir = BaseClass::outputDirectory().name() + "/";
        kvs::StampTimerList timer_list;
        timer_list.push( m_sim_timer );
        timer_list.push( m_cnv_timer );
        timer_list.push( m_vis_timer );
        if ( !timer_list.write( subdir + "proc_time_" + rank + ".csv" ) ) return false;

        // For root node
        auto sim_time_min = m_sim_timer; sim_time_min.reduceMin();
        auto sim_time_max = m_sim_timer; sim_time_max.reduceMax();
        auto sim_time_ave = m_sim_timer; sim_time_ave.reduceAve();
        auto cnv_time_min = m_cnv_timer; cnv_time_min.reduceMin();
        auto cnv_time_max = m_cnv_timer; cnv_time_max.reduceMax();
        auto cnv_time_ave = m_cnv_timer; cnv_time_ave.reduceAve();
        auto vis_time_min = m_vis_timer; vis_time_min.reduceMin();
        auto vis_time_max = m_vis_timer; vis_time_max.reduceMax();
        auto vis_time_ave = m_vis_timer; vis_time_ave.reduceAve();

        if ( !this->world().isRoot() ) return true;

        sim_time_min.setTitle( "Sim time (min)" );
        sim_time_max.setTitle( "Sim time (max)" );
        sim_time_ave.setTitle( "Sim time (ave)" );
        cnv_time_min.setTitle( "Cnv time (min)" );
        cnv_time_max.setTitle( "Cnv time (max)" );
        cnv_time_ave.setTitle( "Cnv time (ave)" );
        vis_time_min.setTitle( "Vis time (min)" );
        vis_time_max.setTitle( "Vis time (max)" );
        vis_time_ave.setTitle( "Vis time (ave)" );

        timer_list.clear();
        timer_list.push( sim_time_min );
        timer_list.push( sim_time_max );
        timer_list.push( sim_time_ave );
        timer_list.push( cnv_time_min );
        timer_list.push( cnv_time_max );
        timer_list.push( cnv_time_ave );
        timer_list.push( vis_time_min );
        timer_list.push( vis_time_max );
        timer_list.push( vis_time_ave );

        const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
        return timer_list.write( basedir + "proc_time.csv" );
    }
};

inline InSituVis::Pipeline InSituVis::OrthoSlice()
{
    return [&] ( Screen& screen, Object& object )
    {
        auto& volume = Volume::DownCast( object );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();
        //auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        t.setRange( min_value, max_value );

        // Create new slice objects.
        auto py = ( volume.minObjectCoord().y() + volume.maxObjectCoord().y() ) * 0.5f;
        auto ay = kvs::OrthoSlice::YAxis;
        auto* object_y = new kvs::OrthoSlice( &volume, py, ay, t );
        object_y->setName( volume.name() + "ObjectY");

        auto pz = ( volume.minObjectCoord().z() + volume.maxObjectCoord().z() ) * 0.5f;
        auto az = kvs::OrthoSlice::ZAxis;
        auto* object_z = new kvs::OrthoSlice( &volume, pz, az, t );
        object_z->setName( volume.name() + "ObjectZ");

        kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "ObjectY") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "ObjectY", object_y );
            screen.scene()->replaceObject( volume.name() + "ObjectZ", object_z );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer_y = new kvs::glsl::PolygonRenderer();
            auto* renderer_z = new kvs::glsl::PolygonRenderer();
            renderer_y->setTwoSideLightingEnabled( true );
            renderer_z->setTwoSideLightingEnabled( true );
            screen.registerObject( object_y, renderer_y );
            screen.registerObject( object_z, renderer_z );
        }
    };
}

inline InSituVis::Pipeline InSituVis::Isosurface()
{
    return [&] ( Screen& screen, Object& object )
    {
        auto& volume = Volume::DownCast( object );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();
        //auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        t.setRange( min_value, max_value );

        // Create new object
        auto i = kvs::Math::Mix( min_value, max_value, 0.5 );
        auto n = kvs::Isosurface::PolygonNormal;
        auto d = true;
        auto* surface = new kvs::Isosurface( &volume, i, n, d, t );
        surface->setName( volume.name() + "Object");

        // Register object and renderer to screen
        kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", surface );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer = new kvs::glsl::PolygonRenderer();
            renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( surface, renderer );
        }
    };
}

inline InSituVis::Pipeline InSituVis::Surface( const kvs::mpi::Communicator& world )
{
    return [world] ( Screen& screen, Object& object )
    {
        auto& volume = Volume::DownCast( object );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        const auto cmap = kvs::ColorMap::BrewerSpectral();
        const auto ratio = float( world.rank() ) / ( world.size() - 1 );
        const auto index = kvs::Math::Round( ( cmap.resolution() - 1 ) * ratio );
        const auto color = cmap[ index ];

        auto* faces = new kvs::ExternalFaces( &volume );
        faces->setName( volume.name() + "Object");
        faces->setColor( color );

        kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", faces );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer = new kvs::glsl::PolygonRenderer();
            renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( faces, renderer );
        }
    };
}

#if defined( IN_SITU_VIS__STOCHASTIC_RENDERING )
inline InSituVis::Pipeline InSituVis::ParticleBasedRendering( const size_t repeats )
{
    return [repeats] ( Screen& screen, Object& object )
    {
        auto& volume = Volume::DownCast( object );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();

        //auto c = kvs::ColorMap::CoolWarm( 256 );
        auto c = kvs::ColorMap::BrewerSpectral( 256 );
        auto o = kvs::OpacityMap( 256 );
        o.addPoint(   0, 0 );
        o.addPoint(  10, 0 );
        o.addPoint( 255, 1 );
        o.create();
        auto t = kvs::TransferFunction( c, o );
        //auto t = kvs::TransferFunction( c );
        t.setRange( min_value, max_value );

        // Particle generation.
        using Sampler = kvs::CellByCellMetropolisSampling;
        const auto* camera = screen.scene()->camera();
        const auto step = 0.5f / 1000.0f;
        auto* point = new Sampler( camera, &volume, repeats, step, t );
        point->setName( volume.name() + "Object");

        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            point->setMinMaxObjectCoords( min_coord, max_coord );
            point->setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Register object and renderer to screen
        //kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", point );
        }
        else
        {
            // Register the objects with renderer.
            auto* point_renderer = new kvs::glsl::ParticleBasedRenderer();
            point_renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( point, point_renderer );
        }
    };
}
#endif

} // end of namspace local
