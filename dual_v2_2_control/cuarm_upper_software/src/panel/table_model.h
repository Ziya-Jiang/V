#ifndef TABLE_MODEL_H
#define TABLE_MODEL_H
#include <QAbstractItemModel>
#include <QStringList>
#include <QVariant>
#include <Eigen/Geometry>

#include "robot_platform_utils/cpp/include/eigen_vector.h"
class TabelModel:public QAbstractTableModel{
    public:
        TabelModel(int row, int col, const QStringList& row_header, const QStringList& col_header, QObject *parent = 0);
        void populateData(const Eigen::MatrixXf&pose);
        int rowCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
        int columnCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
        QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
        QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    private:
        Eigen::MatrixXf table_data_;
        int row_, col_;
        QStringList row_header_;
        QStringList col_header_;
};

#endif