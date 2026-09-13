"""erh:isaac-sim:scene-finalizer - signals completed scene population.

Attributes:
  resources (list[string], required) - every scene-populating component that
                                        must finish before the first render
"""

from typing import ClassVar, Mapping, Sequence, Tuple

from typing_extensions import Self
from viam.components.generic import Generic
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict

from .. import FAMILY, NAMESPACE
from ..sim_manager import SimManager


class IsaacSceneFinalizer(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "scene-finalizer")

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        finalizer = cls(config.name)
        finalizer.reconfigure(config, dependencies)
        return finalizer

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        resources = struct_to_dict(config.attributes).get("resources")
        if not isinstance(resources, list) or not all(
            isinstance(resource, str) and resource for resource in resources
        ):
            raise ValueError(
                f'{config.name}: set "resources" to a list of scene-populating component names'
            )
        return resources, []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        SimManager.get().finalize_scene(self.name)
