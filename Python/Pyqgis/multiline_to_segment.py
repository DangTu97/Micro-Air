
layers = QgsProject.instance().mapLayersByName('multilines')
layer = layers[0]
layer_provider = layer.dataProvider()

feat = QgsFeature()
#point_layer = QgsVectorLayer("Point?crs=epsg:4326", "segment_layer", "memory")
segment_layer = QgsVectorLayer("Linestring?crs=EPSG:4326","segment_layer","memory")
segment_provider = segment_layer.dataProvider()
segment_provider.addAttributes([QgsField("start_x", QVariant.Double)])
segment_provider.addAttributes([QgsField("start_y", QVariant.Double)])
segment_provider.addAttributes([QgsField("end_x", QVariant.Double)])
segment_provider.addAttributes([QgsField("end_y", QVariant.Double)])
segment_layer.updateFields()
idx1 = segment_layer.fields().indexFromName("start_x")
idx2 = segment_layer.fields().indexFromName("start_y")
idx3 = segment_layer.fields().indexFromName("end_x")
idx4 = segment_layer.fields().indexFromName("end_y")


for feature in layer.getFeatures():
    geom = feature.geometry()
    print(geom)
    geomSingleType = QgsWkbTypes.isSingleType(geom.wkbType())
    if geom.type() == QgsWkbTypes.LineGeometry:
        if geomSingleType:
            my_geom = geom.asPolyline()
            start_point = my_geom[0]
            end_point = my_geom[-1]
        else:
            my_geom = geom.asMultiPolyline()
            my_geom = my_geom[0]
            
    for i in range(len(my_geom) - 1):
        start_point = QgsPoint(my_geom[i])
        end_point = QgsPoint(my_geom[i+1])
        start_x = start_point.x()
        start_y = start_point.y()
        end_x = end_point.x()
        end_y = end_point.y()
        
        feat.setGeometry(QgsGeometry.fromPolyline([start_point, end_point]))
        feat.setAttributes([start_x, start_y, end_x, end_y])
        segment_provider.addFeatures([feat])

QgsProject.instance().addMapLayer(segment_layer)