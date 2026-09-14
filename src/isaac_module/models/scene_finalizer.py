"""erh:isaac-sim:scene-finalizer - signals completed scene population."""

from typing import ClassVar, Mapping

from typing_extensions import Self
from viam.components.generic import Generic
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily

from .. import FAMILY, NAMESPACE
from ..sim_manager import SimManager


class IsaacSceneFinalizer(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "scene-finalizer")

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        finalizer = cls(config.name)
        SimManager.get().finalize_scene()
        return finalizer
