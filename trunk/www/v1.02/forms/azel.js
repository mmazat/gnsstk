  var x;
  var y;
  var z;

  function lat_dms(latitude)
  {    
    var lat = latitude;
    var sign = "N";
    if( lat < 0 )
    {
      lat = -lat;
      sign = "S";
    }
    var d = Math.floor(lat);
    var m = Math.floor((lat-d)*60.0);
    var s = (lat-d-m/60.0)*3600.0;
    var dmsv;
    if( d < 10 )
    {
      if( m < 10 )
      {
        dmsv = sign + " 0" + d.toString() + "\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
      else
      {
        dmsv = sign + " 0" + d.toString() + "\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
    }
    else
    {
      if( m < 10 )
      {
        dmsv = sign + " " + d.toString() + "\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
      else
      {
        dmsv = sign + " " + d.toString() + "\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
    }
    return dmsv;
    document.nev.ref_lat_dms.value = dmsv;
  }

  function get_lat1_dms()
  {
    var latitude = document.azel.lat1.value;
    document.azel.lat1dms.value = lat_dms(latitude);
  }
  function get_lat2_dms()
  {
    var latitude = document.azel.lat2.value;
    document.azel.lat2dms.value = lat_dms(latitude);
  }


  function lon_dms(longitude)
  { 
    var lon = longitude;
    var sign = "E";
    if( lon < 0 )
    {
      lon = -lon;
      sign = "W";
    }
    var d = Math.floor(lon);
    var m = Math.floor((lon-d)*60.0);
    var s = (lon-d-m/60.0)*3600.0;
    var dmsv;
    if( d < 10 )
    {
      if( m < 10 )
      {
        dmsv = sign + " 0" + d.toString() + "\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
      else
      {
        dmsv = sign + " 0" + d.toString() + "\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
    }
    else
    {
      if( m < 10 )
      {
        dmsv = sign + " " + d.toString() + "\xB0 0" + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
      else
      {
        dmsv = sign + " " + d.toString() + "\xB0 " + m.toString() + "' " + s.toFixed(6).toString() +"\"";
      }
    }
    return dmsv;
  }

  function get_lon1_dms()
  {
    var longitude = document.azel.lon1.value;
    document.azel.lon1dms.value = lon_dms(longitude);
  }
  function get_lon2_dms()
  {
    var longitude = document.azel.lon2.value;
    document.azel.lon2dms.value = lon_dms(longitude);
  }
  

  function llh2xyz(latitude,longitude,height)
  {
    // Code adapted from code geodesy.c from the GNSS Essentials library for WGS84 only
    var HALFPI = Math.PI/2.0;
    var a = 6378137.0; // semi-major axis of reference ellipse [m]
    var e2 = 0.00669437999014132; // first eccentricity of reference ellipse []
    var N;      // prime vertical radius of curvature [m]
    var sinlat; // sin of the latitude
    var dtmp;   // temp

    
    // check for valid latitude out of range
    if( latitude > HALFPI || latitude < -HALFPI )
    {
      x = 0.0;
      y = 0.0;
      z = 0.0;      
    }
    else
    {
      sinlat = Math.sin( latitude );
      N = a / Math.sqrt( 1.0 - e2 * sinlat*sinlat );
      dtmp = (N + Number(height)) * Math.cos(latitude);
      x = dtmp * Math.cos(longitude);
      y = dtmp * Math.sin(longitude);
      z = ( (1.0 - e2)*N + Number(height) ) * sinlat;
    }
  }

  function llh2xyz_point1()
  {
    var lat = document.azel.lat1.value * Math.PI/180.0;
    var lon = document.azel.lon1.value * Math.PI/180.0;
    var hgt = document.azel.hgt1.value;
    llh2xyz( lat, lon, hgt );
    document.azel.x1.value = x;
    document.azel.y1.value = y;
    document.azel.z1.value = z;
  }

  function llh2xyz_point2()
  {
    var lat = document.azel.lat2.value * Math.PI/180.0;
    var lon = document.azel.lon2.value * Math.PI/180.0;
    var hgt = document.azel.hgt2.value;
    llh2xyz( lat, lon, hgt );
    document.azel.x2.value = x;
    document.azel.y2.value = y;
    document.azel.z2.value = z;
  }
